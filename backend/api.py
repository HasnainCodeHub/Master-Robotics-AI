import asyncio
from typing import Dict, Any
from mangum import Mangum
from app.main import app

# Create the Mangum adapter for ASGI apps
handler = Mangum(app)

def main(event, context):
    """
    AWS Lambda handler for the FastAPI application.
    This is needed for Vercel's serverless functions.
    """
    return handler(event, context)

# Export the handler for Vercel
app_handler = handler