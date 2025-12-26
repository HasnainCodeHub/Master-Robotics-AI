# ui-architecture

## Purpose
Provide structural rules for composing frontend layouts so that pages, modals, overlays, and components render predictably and professionally.

## When to Use
Use this skill when:
- Designing page layouts
- Introducing modals or dialogs
- Separating public vs authenticated UI
- Fixing overlapping or broken UI layers

## How It Works
- Enforces layout hierarchy (App Shell → Page → Section → Component)
- Separates modal layers from page content
- Prevents rendering UI elements outside their intended containers
- Ensures navigation targets real routes, not overlays

## Output
- Stable page layouts
- Predictable navigation behavior
- Clean separation between content and overlays
