"""
Pydantic schemas for authentication API

Handles request/response validation and serialization.
"""

from pydantic import BaseModel, EmailStr, Field, validator
from typing import Optional
from datetime import datetime
from app.auth.models import SkillLevel


# User Schemas
class UserBase(BaseModel):
    """Base user schema"""
    email: EmailStr


class UserCreate(UserBase):
    """Schema for user registration"""
    password: str = Field(..., min_length=8, description="Password must be at least 8 characters")
    name: str = Field(..., min_length=1, max_length=255)
    skill_level: Optional[SkillLevel] = SkillLevel.beginner
    learning_goals: Optional[str] = None
    prior_experience: Optional[str] = None


class UserResponse(UserBase):
    """Schema for user response"""
    id: int
    is_active: bool
    created_at: datetime
    last_login_at: Optional[datetime]

    class Config:
        from_attributes = True


# Learning Profile Schemas
class LearningProfileCreate(BaseModel):
    """Schema for creating learning profile"""
    name: str = Field(..., min_length=1, max_length=255)
    skill_level: SkillLevel = SkillLevel.beginner
    learning_goals: Optional[str] = None
    prior_experience: Optional[str] = None


class LearningProfileUpdate(BaseModel):
    """Schema for updating learning profile"""
    name: Optional[str] = Field(None, min_length=1, max_length=255)
    skill_level: Optional[SkillLevel] = None
    learning_goals: Optional[str] = None
    prior_experience: Optional[str] = None


class LearningProfileResponse(BaseModel):
    """Schema for learning profile response"""
    id: int
    user_id: int
    name: str
    skill_level: SkillLevel
    learning_goals: Optional[str]
    prior_experience: Optional[str]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


# Authentication Schemas
class LoginRequest(BaseModel):
    """Schema for login request"""
    email: EmailStr
    password: str
    remember_me: bool = False


class RegisterRequest(UserCreate):
    """Schema for registration (extends UserCreate)"""
    pass


class Token(BaseModel):
    """Schema for JWT token"""
    access_token: str
    token_type: str = "bearer"


class AuthResponse(BaseModel):
    """Schema for authentication response"""
    access_token: str
    token_type: str = "bearer"
    user: UserResponse
    profile: Optional[LearningProfileResponse]


class PasswordResetRequest(BaseModel):
    """Schema for password reset request"""
    email: EmailStr


class PasswordResetConfirm(BaseModel):
    """Schema for password reset confirmation"""
    token: str
    new_password: str = Field(..., min_length=8)
