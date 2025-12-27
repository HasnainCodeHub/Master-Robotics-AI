"""API route modules."""

from fastapi import APIRouter

from app.api.routes import auth, chat, content, health, translate

api_router = APIRouter()

# Include all route modules
api_router.include_router(auth.router, prefix="/api/auth", tags=["Authentication"])
api_router.include_router(chat.router, prefix="/api", tags=["Chat"])
api_router.include_router(content.router, prefix="/api", tags=["Content"])
api_router.include_router(translate.router, prefix="/api", tags=["Translation"])
api_router.include_router(health.router, prefix="/api", tags=["System"])
