# runtime-safety

## Purpose
Prevent frontend runtime crashes and undefined behavior in browser environments.

## When to Use
Use this skill when:
- Debugging blank screens
- Fixing transient browser errors
- Integrating SDKs or environment variables
- Migrating server logic to frontend

## How It Works
- Detects Node-only globals in browser code
- Guards access to environment variables
- Enforces error boundaries
- Adds defensive checks around async operations

## Output
- No silent crashes
- Visible and traceable runtime errors
- Stable browser execution
