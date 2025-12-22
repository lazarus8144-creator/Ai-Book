"""
CSRF Protection Middleware using Double-Submit Cookie Pattern

This middleware implements CSRF protection for state-changing requests (POST, PUT, DELETE, PATCH).
Uses the double-submit cookie pattern where:
1. Server sends a CSRF token in a cookie
2. Client must send the same token in a request header
3. Server validates that both tokens match

Safe methods (GET, HEAD, OPTIONS) are exempt from CSRF checks.
"""

from fastapi import Request, HTTPException, status
from fastapi.responses import Response
from starlette.middleware.base import BaseHTTPMiddleware
import secrets
import logging

logger = logging.getLogger(__name__)

# CSRF token cookie name
CSRF_COOKIE_NAME = "csrf_token"
# CSRF token header name
CSRF_HEADER_NAME = "X-CSRF-Token"

# Methods that require CSRF protection (state-changing methods)
PROTECTED_METHODS = {"POST", "PUT", "DELETE", "PATCH"}

# Paths that are exempt from CSRF protection
CSRF_EXEMPT_PATHS = {
    "/docs",
    "/redoc",
    "/openapi.json",
    "/api/v1/health",
}


def generate_csrf_token() -> str:
    """
    Generate a cryptographically secure CSRF token

    Returns:
        str: 32-byte URL-safe token
    """
    return secrets.token_urlsafe(32)


class CSRFMiddleware(BaseHTTPMiddleware):
    """
    CSRF protection middleware using double-submit cookie pattern

    For protected methods (POST, PUT, DELETE, PATCH):
    - Validates that CSRF token in cookie matches token in header
    - Returns 403 Forbidden if tokens don't match or are missing

    For safe methods (GET, HEAD, OPTIONS):
    - No validation required
    - Sets CSRF token cookie if not present
    """

    async def dispatch(self, request: Request, call_next):
        """
        Process the request and apply CSRF protection

        Args:
            request: The incoming request
            call_next: The next middleware/endpoint in the chain

        Returns:
            Response with CSRF token cookie

        Raises:
            HTTPException: 403 if CSRF validation fails
        """
        # Get existing CSRF token from cookie
        csrf_token = request.cookies.get(CSRF_COOKIE_NAME)

        # Check if path is exempt
        path = request.url.path
        is_exempt = any(path.startswith(exempt_path) for exempt_path in CSRF_EXEMPT_PATHS)

        # If method requires CSRF protection and path is not exempt
        if request.method in PROTECTED_METHODS and not is_exempt:
            # Get CSRF token from header
            csrf_header = request.headers.get(CSRF_HEADER_NAME)

            # Validate CSRF tokens
            if not csrf_token or not csrf_header:
                logger.warning(
                    f"CSRF validation failed: Missing token. "
                    f"Method: {request.method}, Path: {path}"
                )
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="CSRF token missing. Include CSRF token in both cookie and header."
                )

            if csrf_token != csrf_header:
                logger.warning(
                    f"CSRF validation failed: Token mismatch. "
                    f"Method: {request.method}, Path: {path}"
                )
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="CSRF token validation failed. Token mismatch."
                )

            logger.debug(f"CSRF validation passed for {request.method} {path}")

        # Process the request
        response: Response = await call_next(request)

        # Generate new CSRF token if not present
        if not csrf_token:
            csrf_token = generate_csrf_token()
            logger.debug(f"Generated new CSRF token for {path}")

        # Set CSRF token in cookie (always refresh)
        # Use SameSite=Lax for CSRF protection + usability
        response.set_cookie(
            key=CSRF_COOKIE_NAME,
            value=csrf_token,
            httponly=False,  # Client needs to read this for header
            secure=request.url.scheme == "https",  # HTTPS only in production
            samesite="lax",
            max_age=86400,  # 24 hours
            path="/",
        )

        return response


def get_csrf_token(request: Request) -> str:
    """
    Get CSRF token from request cookie

    Args:
        request: The incoming request

    Returns:
        str: CSRF token or empty string if not present
    """
    return request.cookies.get(CSRF_COOKIE_NAME, "")
