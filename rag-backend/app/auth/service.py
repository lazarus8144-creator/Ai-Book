"""
Authentication service layer

Handles business logic for user registration, login, profile management.
"""

from sqlalchemy.orm import Session
from sqlalchemy import select
from datetime import datetime, timedelta
from typing import Optional
from app.auth.models import User, LearningProfile, Session as UserSession
from app.auth.schemas import UserCreate, LoginRequest, LearningProfileUpdate
from app.auth.security import hash_password, verify_password, create_access_token
from fastapi import HTTPException, status


class AuthService:
    """Service class for authentication operations"""
    
    @staticmethod
    def register(db: Session, user_data: UserCreate) -> tuple[User, str]:
        """
        Register a new user
        
        Args:
            db: Database session
            user_data: User registration data
            
        Returns:
            Tuple of (User, access_token)
            
        Raises:
            HTTPException: If email already exists
        """
        # Check if email already exists
        existing_user = db.execute(
            select(User).where(User.email == user_data.email)
        ).scalar_one_or_none()
        
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="Email already registered"
            )
        
        # Create user with hashed password
        user = User(
            email=user_data.email,
            password_hash=hash_password(user_data.password),
            is_active=True,
            last_login_at=datetime.utcnow()
        )
        
        db.add(user)
        db.flush()  # Get user.id without committing
        
        # Create learning profile
        profile = LearningProfile(
            user_id=user.id,
            name=user_data.name,
            skill_level=user_data.skill_level,
            learning_goals=user_data.learning_goals,
            prior_experience=user_data.prior_experience
        )
        
        db.add(profile)
        db.commit()
        db.refresh(user)
        db.refresh(profile)
        
        # Generate JWT token
        access_token = create_access_token(
            data={"sub": user.email, "user_id": user.id}
        )
        
        return user, access_token
    
    @staticmethod
    def login(db: Session, login_data: LoginRequest) -> tuple[User, str]:
        """
        Authenticate user and create session
        
        Args:
            db: Database session
            login_data: Login credentials
            
        Returns:
            Tuple of (User, access_token)
            
        Raises:
            HTTPException: If credentials are invalid
        """
        # Find user by email
        user = db.execute(
            select(User).where(User.email == login_data.email)
        ).scalar_one_or_none()
        
        if not user or not verify_password(login_data.password, user.password_hash):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password",
                headers={"WWW-Authenticate": "Bearer"}
            )
        
        if not user.is_active:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Account is inactive"
            )
        
        # Update last login
        user.last_login_at = datetime.utcnow()
        db.commit()

        # Generate JWT token with extended expiry if remember_me
        access_token = create_access_token(
            data={"sub": user.email, "user_id": user.id},
            remember_me=login_data.remember_me
        )

        return user, access_token
    
    @staticmethod
    def get_user_by_email(db: Session, email: str) -> Optional[User]:
        """Get user by email"""
        return db.execute(
            select(User).where(User.email == email)
        ).scalar_one_or_none()
    
    @staticmethod
    def get_user_by_id(db: Session, user_id: int) -> Optional[User]:
        """Get user by ID"""
        return db.execute(
            select(User).where(User.id == user_id)
        ).scalar_one_or_none()
    
    @staticmethod
    def update_profile(
        db: Session, 
        user_id: int, 
        profile_data: LearningProfileUpdate
    ) -> LearningProfile:
        """
        Update user's learning profile
        
        Args:
            db: Database session
            user_id: User ID
            profile_data: Profile update data
            
        Returns:
            Updated learning profile
        """
        profile = db.execute(
            select(LearningProfile).where(LearningProfile.user_id == user_id)
        ).scalar_one_or_none()
        
        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Profile not found"
            )
        
        # Update only provided fields
        update_data = profile_data.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(profile, field, value)
        
        db.commit()
        db.refresh(profile)
        
        return profile
