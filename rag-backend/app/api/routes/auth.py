"""
Authentication API endpoints

Handles user registration, login, logout, and session management.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from slowapi import Limiter
from slowapi.util import get_remote_address
from app.database import get_db
from app.auth.schemas import (
    UserCreate,
    LoginRequest,
    AuthResponse,
    UserResponse
)
from app.auth.service import AuthService
from app.auth.dependencies import get_current_user
from app.auth.models import User

# Create router
router = APIRouter(prefix="/api/v1/auth", tags=["authentication"])

# Rate limiter
limiter = Limiter(key_func=get_remote_address)


@router.post("/register", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
@limiter.limit("3/hour")
async def register(
    request: Request,
    user_data: UserCreate,
    db: Session = Depends(get_db)
):
    """
    Register a new user account

    Creates a new user with hashed password and learning profile,
    then returns JWT access token for immediate login.

    Rate limit: 3 registrations per hour per IP

    Args:
        user_data: User registration data
        db: Database session

    Returns:
        AuthResponse: Access token and user profile

    Raises:
        HTTPException 409: Email already registered
    """
    user, access_token = AuthService.register(db, user_data)

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserResponse(
            id=user.id,
            email=user.email,
            is_active=user.is_active,
            created_at=user.created_at,
            profile=user.learning_profile
        )
    )


@router.post("/login", response_model=AuthResponse)
@limiter.limit("5/15minutes")
async def login(
    request: Request,
    login_data: LoginRequest,
    db: Session = Depends(get_db)
):
    """
    Authenticate user and create session

    Validates credentials and returns JWT access token.

    Rate limit: 5 login attempts per 15 minutes per IP

    Args:
        login_data: Login credentials
        db: Database session

    Returns:
        AuthResponse: Access token and user profile

    Raises:
        HTTPException 401: Invalid credentials
        HTTPException 403: Account inactive
    """
    user, access_token = AuthService.login(db, login_data)

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserResponse(
            id=user.id,
            email=user.email,
            is_active=user.is_active,
            created_at=user.created_at,
            profile=user.learning_profile
        )
    )


@router.post("/logout", status_code=status.HTTP_204_NO_CONTENT)
async def logout(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Logout current user and invalidate session

    Note: For stateless JWT, logout is handled client-side by
    removing the token. This endpoint is provided for consistency
    and can be extended to blacklist tokens if needed.

    Args:
        current_user: Authenticated user
        db: Database session

    Returns:
        No content (204)
    """
    # For stateless JWT, no server-side action needed
    # Client should remove token from storage
    # Future: Implement token blacklist if required
    return


@router.post("/refresh", response_model=AuthResponse)
async def refresh_token(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Refresh JWT access token

    Generates a new token with extended expiration for authenticated users.
    Used to maintain persistent sessions without re-login.

    Args:
        current_user: Authenticated user from current JWT
        db: Database session

    Returns:
        AuthResponse: New access token and user profile

    Raises:
        HTTPException 401: If token is invalid or expired
    """
    from app.auth.security import create_access_token

    # Generate new token with 30-day expiry (refresh always uses extended expiry)
    access_token = create_access_token(
        data={"sub": current_user.email, "user_id": current_user.id},
        remember_me=True  # Refresh always extends session
    )

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserResponse(
            id=current_user.id,
            email=current_user.email,
            is_active=current_user.is_active,
            created_at=current_user.created_at,
            profile=current_user.learning_profile
        )
    )


@router.get("/me", response_model=UserResponse)
async def get_current_user_profile(
    current_user: User = Depends(get_current_user)
):
    """
    Get current authenticated user's profile

    Args:
        current_user: Authenticated user from JWT token

    Returns:
        UserResponse: User profile data
    """
    return UserResponse(
        id=current_user.id,
        email=current_user.email,
        is_active=current_user.is_active,
        created_at=current_user.created_at,
        profile=current_user.learning_profile
    )
