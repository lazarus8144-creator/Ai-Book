"""
Authentication dependencies for FastAPI

Provides dependency injection functions for authentication and authorization.
"""

from fastapi import Depends, HTTPException, status, Request, Cookie
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from jose import JWTError, jwt
from typing import Optional
from app.database import get_db
from app.auth.models import User
from app.auth.service import AuthService
from app.config import settings

# HTTP Bearer token security scheme (optional for cookie fallback)
security = HTTPBearer(auto_error=False)


async def get_current_user(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    access_token: Optional[str] = Cookie(default=None),
    db: Session = Depends(get_db)
) -> User:
    """
    Dependency to get the current authenticated user from JWT token

    Supports both cookie-based authentication (for browsers) and
    Authorization header (for API clients).

    Args:
        request: FastAPI request object
        credentials: Optional HTTP Bearer token from Authorization header
        access_token: Optional JWT token from httpOnly cookie
        db: Database session

    Returns:
        User: Authenticated user object

    Raises:
        HTTPException: If token is invalid or user not found
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"}
    )

    # Try to get token from cookie first, then Authorization header
    token = None
    if access_token:
        token = access_token
    elif credentials:
        token = credentials.credentials

    if not token:
        raise credentials_exception

    try:
        # Decode JWT token
        payload = jwt.decode(
            token,
            settings.jwt_secret_key,
            algorithms=[settings.jwt_algorithm]
        )

        # Extract user email from token
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception

    except JWTError:
        raise credentials_exception

    # Get user from database
    user = AuthService.get_user_by_email(db, email=email)
    if user is None:
        raise credentials_exception

    # Check if user is active
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Inactive user"
        )

    return user


async def get_current_active_user(
    current_user: User = Depends(get_current_user)
) -> User:
    """
    Dependency to ensure user is active

    Args:
        current_user: User from get_current_user dependency

    Returns:
        User: Active user object

    Raises:
        HTTPException: If user is inactive
    """
    if not current_user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Inactive user"
        )
    return current_user
