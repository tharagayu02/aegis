from datetime import datetime
from decimal import Decimal

from sqlalchemy import inspect, select, text
from sqlalchemy.engine import make_url

from backend.app.database import DATABASE_URL, SessionLocal, create_mysql_server_engine, engine
from backend.app.models import Alert, Base, Drone, FlightPath, Telemetry, Threat

INITIAL_DRONES = (
    ("RECON-01", "Recon Alpha", 11.1089, 77.3456, 96.0),
    ("RECON-02", "Recon Bravo", 11.1200, 77.3600, 88.0),
    ("RECON-03", "Recon Charlie", 11.0950, 77.3300, 91.0),
    ("RECON-04", "Recon Delta", 11.1100, 77.3700, 84.0),
)

INITIAL_THREATS = (
    {
        "threat_id": "THREAT-404",
        "detected_by": "CIT-SURVEILLANCE",
        "threat_type": "Restricted Demonstration Zone",
        "severity": "HIGH",
        "min_latitude": 11.10865,
        "max_latitude": 11.10925,
        "min_longitude": 77.34625,
        "max_longitude": 77.34685,
    },
    {
        "threat_id": "THREAT-405",
        "detected_by": "CIT-SURVEILLANCE",
        "threat_type": "Restricted Perimeter",
        "severity": "MEDIUM",
        "min_latitude": 11.1150,
        "max_latitude": 11.1160,
        "min_longitude": 77.3620,
        "max_longitude": 77.3630,
    },
)


def ensure_database_exists() -> None:
    url = make_url(DATABASE_URL)
    database_name = url.database
    if not database_name:
        raise RuntimeError("DATABASE_URL must include the target database name drone_toc.")
    if database_name != "drone_toc":
        raise RuntimeError(
            f"Refusing to initialize unexpected database {database_name!r}; expected 'drone_toc'."
        )

    server_engine = create_mysql_server_engine()
    try:
        with server_engine.begin() as connection:
            quoted_name = connection.dialect.identifier_preparer.quote(database_name)
            connection.execute(
                text(
                    f"CREATE DATABASE IF NOT EXISTS {quoted_name} "
                    "CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci"
                )
            )
    finally:
        server_engine.dispose()


def seed_initial_records() -> None:
    with SessionLocal() as session:
        for drone_id, name, latitude, longitude, battery in INITIAL_DRONES:
            if session.scalar(select(Drone.id).where(Drone.drone_id == drone_id)) is None:
                session.add(
                    Drone(
                        drone_id=drone_id,
                        drone_name=name,
                        status="ACTIVE",
                        battery_pct=Decimal(str(battery)),
                        speed=Decimal("0"),
                        latitude=Decimal(str(latitude)),
                        longitude=Decimal(str(longitude)),
                    )
                )
        session.flush()

        for threat_data in INITIAL_THREATS:
            if (
                session.scalar(
                    select(Threat.id).where(Threat.threat_id == threat_data["threat_id"])
                )
                is None
            ):
                session.add(
                    Threat(
                        **{
                            key: Decimal(str(value)) if key.endswith(("latitude", "longitude")) else value
                            for key, value in threat_data.items()
                        },
                        timestamp=datetime.utcnow(),
                        is_active=True,
                    )
                )
        session.commit()


def verify_schema() -> dict[str, list[str]]:
    inspector = inspect(engine)
    expected = {
        "drones": Drone.__table__,
        "telemetry": Telemetry.__table__,
        "threats": Threat.__table__,
        "flight_paths": FlightPath.__table__,
        "alerts": Alert.__table__,
    }
    result: dict[str, list[str]] = {}
    for table_name, table in expected.items():
        actual_columns = {column["name"] for column in inspector.get_columns(table_name)}
        missing = {column.name for column in table.columns} - actual_columns
        if missing:
            raise RuntimeError(
                f"Existing table {table_name!r} is incompatible; missing columns: "
                f"{', '.join(sorted(missing))}. Back up and migrate that table explicitly; "
                "the initializer will not drop or overwrite user data."
            )
        expected_indexes = {
            index.name for index in table.indexes if index.name is not None
        }
        actual_indexes = {
            index["name"] for index in inspector.get_indexes(table_name)
        }
        missing_indexes = expected_indexes - actual_indexes
        if missing_indexes:
            raise RuntimeError(
                f"Existing table {table_name!r} is missing expected indexes: "
                f"{', '.join(sorted(missing_indexes))}. Back up and add the indexes "
                "explicitly; the initializer will not alter existing tables."
            )
        result[table_name] = sorted(actual_columns)
    for table_name in ("telemetry", "flight_paths", "alerts"):
        foreign_keys = inspector.get_foreign_keys(table_name)
        valid_drone_fk = any(
            foreign_key.get("constrained_columns") == ["drone_id"]
            and foreign_key.get("referred_table") == "drones"
            and foreign_key.get("referred_columns") == ["drone_id"]
            for foreign_key in foreign_keys
        )
        if not valid_drone_fk:
            raise RuntimeError(
                f"Existing table {table_name!r} lacks the required foreign key "
                "from drone_id to drones.drone_id. Back up and repair the constraint "
                "explicitly; the initializer will not alter existing tables."
            )
    return result


def main() -> None:
    print("Connecting to MySQL server...")
    ensure_database_exists()
    with engine.connect() as connection:
        server_version = connection.execute(text("SELECT VERSION()")).scalar_one()
        database_name = connection.execute(text("SELECT DATABASE()")).scalar_one()
    if not str(server_version).split("-", maxsplit=1)[0].startswith("8."):
        raise RuntimeError(f"MySQL 8.x is required; connected server reports {server_version}.")

    print(f"MySQL: CONNECTED ({server_version})")
    print(f"Database: {database_name}")
    Base.metadata.create_all(bind=engine, checkfirst=True)
    verify_schema()
    seed_initial_records()

    with SessionLocal() as session:
        drone_count = session.scalar(select(text("count(*)")).select_from(Drone)) or 0
        threat_count = session.scalar(select(text("count(*)")).select_from(Threat)) or 0
    print("DATABASE INITIALIZATION SUCCESSFUL")
    print(f"Tables: {', '.join(Base.metadata.tables)}")
    print(f"Drones: {drone_count}")
    print(f"Threat zones: {threat_count}")
    print("Existing rows were preserved; missing sample records were inserted only.")


if __name__ == "__main__":
    main()
