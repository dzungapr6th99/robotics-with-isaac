# BusinessAnalyse.md

This document describes **business behavior per module**.
It focuses on **what the system does**, **why it exists**, and **how each module behaves**,  
not on technical or architectural details.

> IMPORTANT FOR AI AGENTS  
> - Follow existing modules and patterns  
> - Do NOT invent new flows or rules  
> - If something is unclear, leave TODO instead of guessing

---

## 0) Global Conventions

### 0.1 Purpose of this document
- Explain business meaning and flows per module
- Help developers and AI agents understand:
  - What each module owns
  - How data is created / updated / used
  - Which APIs exist and why

---

### 0.2 Common API types
Most modules will expose:
- **CRUD APIs** (Create / Read / Update / Delete)
- **Business APIs** (non-CRUD, scenario-based)
- **Internal APIs / integrations** (used by other modules or systems)

---

### 0.3 Common flow structure
Each module typically describes:
1. Purpose / meaning
2. Main data
3. Supported APIs
4. Business flows (upload, download, assign, etc.)
5. Business rules & constraints
6. Edge cases & TODOs

---

## 1) Module: Map

### 1.1 Business Meaning
The **Map module** manages navigation maps used by robots.

A map represents the spatial environment in which robots operate.
It is a core resource for navigation, localization, and mission execution.

Maps may be:
- Uploaded by operators or backend systems
- Downloaded by robots

> TODO: clarify supported map formats (e.g. yaml/pgm/png/json/usd)

---

### 1.2 Core Data
Typical map-related data includes:
- Id
- Name
- MinioUrl (url in minio to store map files) 

> TODO: finalize exact fields and constraints

---

### 1.3 APIs (Business View)

#### CRUD APIs
- Create Map 
- Get Map (by id / name)
- Update Map metadata
- Delete Map

These APIs are used mainly by:
- Admin / operator tools
- Internal management services

---

#### Business APIs (Non-CRUD)
- Upload map files
- Assign map for robot 
- Get latest map version
- Validate map before use (optional)

These APIs serve **specific business scenarios**, not generic data management.

---

### 1.4 Business Flows

#### 1.4.1 Upload Map Flow
Typical flow:
1. User/system create map. ( can uploads map file(s) if have attached file)
2. System validates basic format and metadata
3. Files are stored at minio (send stored to minio)
4. Map metadata is saved
5. New map version is created (if applicable)

Business intent:
- Ensure maps are centrally managed
- Enable version tracking and rollback
- Ensure Robots are connected to system via mqtt. If any robot is not connected, list them in response.
> TODO: define overwrite vs versioning behavior

---

#### 1.4.2 Download Map Flow (Robot)
Typical flow:
1. User assigned a robot or list of robots to assigned
2. System checks map existence and availability
3. System created download link or stream
4. System send instantActions downloadMap for robot.

Business intent:
- Robots always use the correct and approved map
- Minimize manual map distribution

> TODO: authentication / authorization rules for robot access

---

### 1.5 Business Rules
Examples:
- A map must exist before it can be downloaded
- Only one “active” map version may be used per environment (if applicable)
- Deleting a map may be restricted if robots are currently using it

> TODO: add rule details when finalized

---

### 1.6 Edge Cases & Notes
- Partial upload failure
- Robot downloads outdated map
- Concurrent map updates
- Storage unavailable

> TODO: define fallback and retry behavior

---

## 2) Module: Point

### 2.1 Business Meaning
Points in map, include coordinates of depot in SLAM map. User will setup points and conbine with routes to create path for Robot and robot must go on the path set up.

---

### 2.2 Core Data
- Id
- MapId
- X
- Y
- PointTypeId

---

### 2.3 APIs
#### CRUD APIs
- Insert
- Update
- Delete
- Get
#### Business APIs (Non-CRUD)
- Algin Point (algin list of point follow coordinate of first and last point)
---

### 2.4 Business Flows
CRUD flow:
- Ensure MapId exist in database

---

### 2.5 Business Rules


---

### 2.6 Edge Cases & TODOs

---


## 3) Module: Route

### 3.1 Business Meaning
Routes in map, include coordinates of depot in SLAM map. User will setup routes and conbine with points to create path for Robot and robot must go on the path set up.

---

### 3.2 Core Data
- Id
- MapId
- FromPointId
- ToPointId

---

### 3.3 APIs
#### CRUD APIs
- Insert
- Update
- Delete
- Get
#### Business APIs (Non-CRUD)

---

### 3.4 Business Flows
CRUD flow:
- Ensure MapId exist in database
- Ensure FromPointId, ToPointId exist in database and Map is contain them.
---

### 3.5 Business Rules


---

### 3.6 Edge Cases & TODOs

---

## 4) Module: Robot

### 4.1 Business Meaning
Robots are assigned to system.

---

### 4.2 Core Data
- Id
- InterfaceName
- Manufacturer
- SerialNumber
- RobotTypeId

---

### 4.3 APIs
#### CRUD APIs
- Insert
- Update
- Delete
- Get
#### Business APIs (Non-CRUD)
---

### 4.4 Business Flows
CRUD flow:
- Ensure Id, SerialNumber are unique in database

---

### 4.5 Business Rules


---

### 4.6 Edge Cases & TODOs

---

## 5) Module: Point

### 5.1 Business Meaning
Points in map, include coordinates of depot in SLAM map. User will setup points and conbine with routes to create path for Robot and robot must go on the path set up.

---

### 5.2 Core Data
- Id
- MapId
- X
- Y
- PointTypeId

---

### 5.3 APIs
#### CRUD APIs
- Insert
- Update
- Delete
- Get
#### Business APIs (Non-CRUD)
- Algin Point (algin list of point follow coordinate of first and last point)
---

### 5.4 Business Flows
CRUD flow:
- Ensure MapId exist in database

---

### 5.5 Business Rules


---

### 5.6 Edge Cases & TODOs

---
