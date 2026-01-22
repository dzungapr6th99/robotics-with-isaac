# Architecture (Clean Architecture - Project Convention)

This repository already has a working architecture.  
AI agents should **follow the existing structure** and **must NOT redesign** or “invent” a new architecture.

We document the architecture using the **project’s convention**:

Controller  
↓  
Business  
↓  
Infrastructure  
↓  
Entities  

> Note: The arrow describes the **typical call flow and responsibility flow** in this codebase.
> Each layer should remain focused on its own job.

---

## 1) Controller Layer (API / Entry Points)
(Precheck + simple validation + request mapping)

Responsibilities:
- Entry points: REST controllers / WebSocket handlers / message consumers / CLI endpoints
- Simple validations: required fields, format checks, range checks
- Map transport models -> business input DTOs
- Call business services / use-cases
- Map business result -> transport response

Rules:
- No business decision logic
- No direct DB / Kafka / MinIO / MQTT / ZeroMQ usage
- Keep it thin and predictable

AI Agent Notes:
- If you need additional validation, keep it **format-level** here.
- Anything “meaningful for business” belongs to Business.

---

## 2) Business Layer (Main Business Logic)
(Core workflows + business rules + module-specific logic)

Responsibilities:
- Business workflows (use-cases / services per module)
- Business validations and rules (including checks that require DB data)
- Orchestrate multiple operations to complete a scenario
- Decide when to persist, publish events, or call external systems

Rules:
- Business logic lives here (not in Controller, not in Infrastructure)
- Business can call Infrastructure through **interfaces** already defined in the project
- No framework-specific code unless the project explicitly allows it

AI Agent Notes:
- Do not move business rules into Infrastructure “because it’s convenient”.
- If adding new features, implement the rule in Business and reuse infra capabilities.

---

## 3) Infrastructure Layer (Technical Capabilities / Integrations)
(DB, messaging, storage, and system integration implementations)

Responsibilities:
- Database access implementations (repositories, transactions, queries)
- Messaging implementations (Kafka / MQ / event bus)
- Object storage (MinIO)
- External HTTP/gRPC clients for internal microservices
- Transport servers/adapters (MQTT / ZeroMQ)
- Logging, monitoring, caching (as needed by project)

Rules:
- Infrastructure should implement **generic, reusable** capabilities
  (CRUD, publish, send, store, query, etc.)
- Avoid module-specific business rules here
- Keep it replaceable (different DB/broker/storage) with minimal changes to Business

AI Agent Notes:
- If a new integration is needed, implement it here and expose it via existing patterns.
- Don’t embed business decisions inside infra implementations.

---

## 4) Entities Layer (Shared Core Models)
(Entities + config/common + mapping classes + message contracts)

Responsibilities:
- Entities / core models
- Config / common types / constants / enums
- Database mapping models (if used in this project)
- Message contracts (events, commands, DTOs shared between layers)
- Common exceptions (non-framework)

Rules:
- Entities should be stable and dependency-light
- Entities should NOT directly access DB, messaging, or external services
- Keep models clean and reusable

AI Agent Notes:
- If you need a new shared type used across layers, put it here.
- Don’t add business workflow logic in Entities.

---

## Special Notes (Important for AI Agents)

### A) Architecture is already implemented
- Do not “refactor into textbook clean architecture”
- Follow existing folder/projects and patterns

### B) Interface-first dependency
- Cross-layer usage should be via interfaces (as the project already does)
- Implementations belong in Infrastructure

### C) Common flows to respect
- Controller: validate format → call Business
- Business: enforce rules → call Infrastructure capabilities → return result
- Infrastructure: execute technical operations → return data/ack
- Entities: define shared models/messages used by the above
- In CRUD rules, both Business and Infra always have base interface (IBaseBL, IBaseDA) and static  interface for each module (IMapBL, IMapDA, ...) if BusinessLayer want work with other microservice instead of database, implement those function in static interface for each module to ensure SOLID principle.
- Other microservice must be description in appsettings.json and implelemt in ConfigData.cs (detail in each module such as bucket with Minio or topic in Kafka)
