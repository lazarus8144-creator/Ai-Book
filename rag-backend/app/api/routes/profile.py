"""
Profile management API endpoints

Handles user profile viewing and editing.
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from app.database import get_db
from app.auth.dependencies import get_current_user
from app.auth.models import User
from app.auth.schemas import LearningProfileUpdate, LearningProfileResponse
from app.auth.service import AuthService

# Create router
router = APIRouter(prefix="/api/v1/profile", tags=["profile"])


@router.get("", response_model=LearningProfileResponse)
async def get_profile(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get current user's learning profile

    Args:
        current_user: Authenticated user
        db: Database session

    Returns:
        LearningProfileResponse: User's learning profile

    Raises:
        HTTPException 404: Profile not found
    """
    if not current_user.learning_profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    return current_user.learning_profile


@router.put("", response_model=LearningProfileResponse)
async def update_profile(
    profile_data: LearningProfileUpdate,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update current user's learning profile

    Allows updating name, skill_level, learning_goals, and prior_experience.
    All fields are optional.

    Args:
        profile_data: Profile update data
        current_user: Authenticated user
        db: Database session

    Returns:
        LearningProfileResponse: Updated learning profile

    Raises:
        HTTPException 404: Profile not found
    """
    updated_profile = AuthService.update_profile(
        db=db,
        user_id=current_user.id,
        profile_data=profile_data
    )

    return updated_profile
