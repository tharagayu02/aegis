# AEGIS Drone Operations TOC

AEGIS is a local full-stack drone simulation dashboard. The browser communicates only with FastAPI; MySQL credentials remain in `backend/.env`. MySQL stores drones, telemetry, bounding-box restricted zones, A* flight paths, and alerts. Threat detection and Haversine distance calculations run in Python; the A* planner uses a local metric grid.

The former PostgreSQL/PostGIS database and installation are not deleted, modified, or migrated by this project. The MySQL `drone_toc` database is a separate database on port 3306. The initializer only creates the database/tables if missing and inserts the four demo drones and two demo restricted zones if their identifiers are absent.

## Prerequisites

- Python 3.11+
- MySQL 8.x Server and command-line client
- Node.js/npm
- A CARTO public map key configured in `frontend/.env.local`

On Windows, MySQL service names vary by installer. Check with `Get-Service *mysql*`; start the installed service from an Administrator PowerShell if necessary, for example `Start-Service MySQL80`. The application does not install or modify the MySQL or PostgreSQL server services.

## 1. Configure MySQL

Copy the backend template and set the actual MySQL root password locally:

```powershell
Copy-Item backend\.env.example backend\.env
notepad backend\.env
```

Set:

```dotenv
DATABASE_URL=mysql+pymysql://root:YOUR_MYSQL_PASSWORD@localhost:3306/drone_toc
API_BASE_URL=http://127.0.0.1:8000
SIMULATOR_INTERVAL_SECONDS=1
```

`backend/.env` is ignored by Git. Do not copy `DATABASE_URL` into any frontend environment file.

If your MySQL user is permitted to create databases, initialization creates `drone_toc` with `CREATE DATABASE IF NOT EXISTS`. Otherwise sign in with an authorized MySQL account and run this once:

```sql
CREATE DATABASE IF NOT EXISTS drone_toc
  CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci;
```

This does not drop or overwrite the database. If a table with an incompatible pre-existing schema is found, initialization stops and explains the missing columns rather than destroying that table.

## 2. Install Python dependencies and initialize tables

From the project root:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m backend.init_db
```

Initialization checks that it connected to MySQL 8 and selected `drone_toc`, creates missing tables and indexes, verifies their columns/indexes, and inserts absent demo records. It does not delete existing rows. Tables are `drones`, `telemetry`, `threats`, `flight_paths`, and `alerts`; latitude/longitude and threat bounding boxes are decimal columns, not PostGIS types.

## 3. Start FastAPI

In a project-root terminal with the virtual environment active:

```powershell
uvicorn backend.app.main:app --reload
```

API documentation: `http://127.0.0.1:8000/docs`. The health endpoint at `http://127.0.0.1:8000/api/health` checks the database connection and required tables.

## 4. Start the simulator

In a second project-root terminal:

```powershell
.\.venv\Scripts\Activate.ps1
python -m backend.simulator
```

The simulator resumes stored positions, moves all four drones gradually, checks each proposed movement against active threat bounding boxes, inserts telemetry to MySQL every second, saves safe A* routes to `flight_paths`, and records threat/reroute/completion alerts in MySQL.

## 5. Configure and start Next.js

Create the frontend's public-only environment file:

```powershell
Copy-Item frontend\.env.example frontend\.env.local
notepad frontend\.env.local
```

Set the API URL and your public CARTO key:

```dotenv
NEXT_PUBLIC_API_BASE_URL=http://127.0.0.1:8000
NEXT_PUBLIC_CARTO_API_KEY=YOUR_CARTO_KEY
```

Then, in a third terminal:

```powershell
cd frontend
npm install
npm run dev
```

Open `http://localhost:3000`. Restart Next.js after editing `.env.local` so its public variables are reloaded. PostgreSQL/MySQL passwords must never be placed in this file.

## Demonstration

1. Start MySQL, then run initialization once.
2. Start FastAPI and wait for `GET /api/health` to report `"status": "ok"`.
3. Start the simulator and frontend. Confirm all four drones and both stored threat zones appear.
4. Enable **DEMO MODE** on the dashboard. The simulator chooses the nearest drone outside an active restricted zone and deterministically steers it toward that zone.
5. The proposed segment is checked before persistence. When it would enter a restricted box, it is rejected; an A* path is calculated, written to `flight_paths`, and displayed as **OPTIMIZED A* ROUTE**. Threat, blocked-route, generated-route, course-corrected, and safe-route events are stored in `alerts`.
6. The drone follows the safe route, then returns to `ACTIVE`. Disable demo mode with the same dashboard control.

No simulated position is fabricated in React, and the simulator does not communicate with real drone hardware. With no active zones, no avoidance event can occur.

## API

- `GET /api/health`
- `GET /api/drones`, `GET /api/drones/{drone_id}`
- `GET /api/telemetry`, `GET /api/telemetry/{drone_id}`, `POST /api/telemetry`
- `GET /api/threats`, `GET /api/threats/active`
- `GET /api/alerts`, `POST /api/alerts`
- `GET /api/drones/{drone_id}/threat-check`
- `GET /api/drones/{drone_id}/nearby-threats` (Haversine distance to active bounding boxes)
- `GET /api/drones/{drone_id}/route`
- `POST /api/drones/{drone_id}/course-correct`
- `GET /api/simulation/status`, `POST /api/simulation/demo`, `POST /api/simulation/state`

## Database verification

Connect with the MySQL client and enter the password when prompted:

```powershell
mysql -u root -p drone_toc
```

Then check the database, seeded rows, route/alert tables, and telemetry movement:

```sql
SELECT DATABASE(), VERSION();
SHOW TABLES;
SELECT drone_id, status, latitude, longitude, battery_pct FROM drones ORDER BY drone_id;
SELECT drone_id, timestamp, latitude, longitude FROM telemetry ORDER BY id DESC LIMIT 12;
SELECT threat_id, severity, min_latitude, max_latitude, min_longitude, max_longitude FROM threats;
SELECT drone_id, path_type, distance, is_safe, created_at FROM flight_paths ORDER BY id DESC LIMIT 5;
SELECT drone_id, alert_type, message, timestamp FROM alerts ORDER BY id DESC LIMIT 10;
```

The existing PostgreSQL installation and database are left untouched. This implementation does not use PostGIS or PostgreSQL drivers.
