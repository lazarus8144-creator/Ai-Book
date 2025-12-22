"""
Password reset API endpoints

Handles forgot password and reset password flows.
"""

from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from slowapi import Limiter
from slowapi.util import get_remote_address
from datetime import datetime, timedelta
import secrets
import logging

from app.database import get_db
from app.auth.schemas import PasswordResetRequest, PasswordResetConfirm
from app.auth.models import PasswordResetToken, User
from app.auth.security import hash_password
from app.auth.service import AuthService

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api/v1/auth", tags=["password-reset"])

# Rate limiter
limiter = Limiter(key_func=get_remote_address)


@router.post("/forgot-password", status_code=status.HTTP_200_OK)
@limiter.limit("5/hour")
async def forgot_password(
    request: Request,
    reset_request: PasswordResetRequest,
    db: Session = Depends(get_db)
):
    """
    Request password reset

    Generates a reset token and logs it (instead of sending email).
    Returns generic success message for security (no user enumeration).

    Rate limit: 5 requests per hour per IP

    Args:
        reset_request: Email address for password reset
        db: Database session

    Returns:
        Success message (always returns 200 even if email doesn't exist)
    """
    # Check if user exists
    user = AuthService.get_user_by_email(db, reset_request.email)

    if user:
        # Generate secure random token
        token = secrets.token_urlsafe(32)
        token_hash = hash_password(token)

        # Create reset token record (expires in 1 hour)
        reset_token = PasswordResetToken(
            email=reset_request.email,
            token_hash=token_hash,
            expires_at=datetime.utcnow() + timedelta(hours=1)
        )

        db.add(reset_token)
        db.commit()

        # Log the reset link (instead of sending email)
        reset_link = f"http://localhost:3000/reset-password?token={token}"
        logger.info(f"""
        PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        = PASSWORD RESET REQUEST
        PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        Email: {reset_request.email}
        Reset Link: {reset_link}
        Token: {token}
        Expires: 1 hour from now
        PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        NOTE: In production, this would be sent via email.
        For development, copy the link above to test password reset.
        PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
        """)

    # Always return success (prevent user enumeration)
    return {
        "message": "If an account exists with that email, a password reset link has been sent.",
        "dev_note": "Check server console for reset link (email not configured)"
    }


@router.post("/reset-password", status_code=status.HTTP_200_OK)
async def reset_password(
    reset_confirm: PasswordResetConfirm,
    db: Session = Depends(get_db)
):
    """
    Reset password using token

    Validates the reset token and updates the user's password.
    Tokens are single-use and expire after 1 hour.

    Args:
        reset_confirm: Reset token and new password
        db: Database session

    Returns:
        Success message

    Raises:
        HTTPException 400: Invalid or expired token
    """
    from app.auth.security import verify_password

    # Find valid reset tokens for this token
    # Check all recent tokens since we need to hash-compare
    one_hour_ago = datetime.utcnow() - timedelta(hours=1)
    potential_tokens = db.query(PasswordResetToken).filter(
        PasswordResetToken.expires_at > datetime.utcnow(),
        PasswordResetToken.used_at.is_(None),
        PasswordResetToken.expires_at > one_hour_ago
    ).all()

    # Find matching token by verifying hash
    reset_token = None
    for token_record in potential_tokens:
        if verify_password(reset_confirm.token, token_record.token_hash):
            reset_token = token_record
            break

    if not reset_token:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired reset token"
        )

    # Get user and update password
    user = AuthService.get_user_by_email(db, reset_token.email)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Update password
    user.password_hash = hash_password(reset_confirm.new_password)

    # Mark token as used
    reset_token.used_at = datetime.utcnow()

    db.commit()

    logger.info(f"Password successfully reset for user: {user.email}")

    return {"message": "Password successfully reset. You can now log in with your new password."}
