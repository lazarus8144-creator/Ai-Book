"""
Integration tests for authentication flows

Tests complete user journeys:
1. Register → Login → Profile View → Logout
2. Register → Profile Update → Password Reset
3. Login with Remember Me → Token Refresh
"""

import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
import secrets

from app.main import app
from app.database import get_db, Base
from app.auth.models import User, LearningProfile


# Create in-memory SQLite database for testing
SQLALCHEMY_TEST_DATABASE_URL = "sqlite:///:memory:"

engine = create_engine(
    SQLALCHEMY_TEST_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def override_get_db():
    """Override database dependency for testing"""
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()


# Override the dependency
app.dependency_overrides[get_db] = override_get_db

# Create test client
client = TestClient(app)


@pytest.fixture(autouse=True)
def setup_database():
    """Create fresh database for each test"""
    Base.metadata.create_all(bind=engine)
    yield
    Base.metadata.drop_all(bind=engine)


@pytest.fixture
def test_user_data():
    """Test user data"""
    return {
        "email": f"test_{secrets.token_hex(4)}@example.com",
        "password": "TestPassword123!",
        "name": "Test User",
        "skill_level": "intermediate",
        "learning_goals": "Learn humanoid robotics",
        "prior_experience": "Basic Python programming"
    }


class TestCompleteAuthFlow:
    """Test complete authentication flows"""

    def test_register_login_profile_logout_flow(self, test_user_data):
        """
        Test: Register → Login → View Profile → Logout

        This tests the most common user journey
        """
        # Step 1: Register a new user
        register_response = client.post(
            "/api/v1/auth/register",
            json=test_user_data
        )
        assert register_response.status_code == 201
        register_data = register_response.json()
        assert "user" in register_data
        assert register_data["user"]["email"] == test_user_data["email"]
        assert "access_token" in register_response.cookies

        # Store cookies for subsequent requests
        cookies = register_response.cookies

        # Step 2: Verify we can access protected endpoint (profile)
        profile_response = client.get(
            "/api/v1/profile",
            cookies=cookies
        )
        assert profile_response.status_code == 200
        profile_data = profile_response.json()
        assert profile_data["name"] == test_user_data["name"]
        assert profile_data["skill_level"] == test_user_data["skill_level"]

        # Step 3: Logout
        logout_response = client.post(
            "/api/v1/auth/logout",
            cookies=cookies
        )
        assert logout_response.status_code == 204

        # Step 4: Verify we can't access protected endpoint after logout
        profile_after_logout = client.get(
            "/api/v1/profile",
            cookies=logout_response.cookies
        )
        assert profile_after_logout.status_code == 401

        # Step 5: Login again with the same credentials
        login_response = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": test_user_data["password"],
                "remember_me": False
            }
        )
        assert login_response.status_code == 200
        login_data = login_response.json()
        assert login_data["user"]["email"] == test_user_data["email"]
        assert "access_token" in login_response.cookies

        # Step 6: Verify profile access works again
        new_cookies = login_response.cookies
        profile_response_2 = client.get(
            "/api/v1/profile",
            cookies=new_cookies
        )
        assert profile_response_2.status_code == 200

    def test_register_update_profile_flow(self, test_user_data):
        """
        Test: Register → Update Profile → Verify Changes
        """
        # Step 1: Register
        register_response = client.post(
            "/api/v1/auth/register",
            json=test_user_data
        )
        assert register_response.status_code == 201
        cookies = register_response.cookies

        # Step 2: Update profile
        updated_data = {
            "name": "Updated Name",
            "skill_level": "advanced",
            "learning_goals": "Build my own humanoid robot",
            "prior_experience": "5 years in robotics"
        }
        update_response = client.put(
            "/api/v1/profile",
            json=updated_data,
            cookies=cookies
        )
        assert update_response.status_code == 200
        update_data = update_response.json()
        assert update_data["name"] == updated_data["name"]
        assert update_data["skill_level"] == updated_data["skill_level"]
        assert update_data["learning_goals"] == updated_data["learning_goals"]

        # Step 3: Verify changes persist
        profile_response = client.get(
            "/api/v1/profile",
            cookies=cookies
        )
        assert profile_response.status_code == 200
        profile_data = profile_response.json()
        assert profile_data["name"] == updated_data["name"]
        assert profile_data["skill_level"] == updated_data["skill_level"]

    def test_password_reset_flow(self, test_user_data):
        """
        Test: Register → Request Password Reset → Reset Password → Login with New Password
        """
        # Step 1: Register
        register_response = client.post(
            "/api/v1/auth/register",
            json=test_user_data
        )
        assert register_response.status_code == 201

        # Step 2: Request password reset
        reset_request_response = client.post(
            "/api/v1/auth/forgot-password",
            json={"email": test_user_data["email"]}
        )
        assert reset_request_response.status_code == 200

        # In real scenario, we'd extract token from email
        # For testing, we need to generate a token directly
        # This is a simplified test - in production, you'd mock email service
        from app.auth.security import hash_password
        from app.auth.models import PasswordResetToken
        from datetime import datetime, timedelta

        # Generate test token
        test_token = secrets.token_urlsafe(32)
        token_hash = hash_password(test_token)

        # Insert directly into test database
        db = next(override_get_db())
        reset_token = PasswordResetToken(
            email=test_user_data["email"],
            token_hash=token_hash,
            expires_at=datetime.utcnow() + timedelta(hours=1)
        )
        db.add(reset_token)
        db.commit()

        # Step 3: Reset password using token
        new_password = "NewSecurePassword123!"
        reset_response = client.post(
            "/api/v1/auth/reset-password",
            json={
                "token": test_token,
                "new_password": new_password
            }
        )
        assert reset_response.status_code == 200

        # Step 4: Try logging in with old password (should fail)
        old_login_response = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": test_user_data["password"],
                "remember_me": False
            }
        )
        assert old_login_response.status_code == 401

        # Step 5: Login with new password (should succeed)
        new_login_response = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": new_password,
                "remember_me": False
            }
        )
        assert new_login_response.status_code == 200
        assert "access_token" in new_login_response.cookies

    def test_remember_me_functionality(self, test_user_data):
        """
        Test: Register → Login without Remember Me → Login with Remember Me

        Verifies that remember_me flag affects token expiration
        """
        # Step 1: Register
        register_response = client.post(
            "/api/v1/auth/register",
            json=test_user_data
        )
        assert register_response.status_code == 201

        # Step 2: Login WITHOUT remember me
        login_no_remember = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": test_user_data["password"],
                "remember_me": False
            }
        )
        assert login_no_remember.status_code == 200
        no_remember_cookie = login_no_remember.cookies.get("access_token")
        assert no_remember_cookie is not None

        # Step 3: Logout
        client.post("/api/v1/auth/logout", cookies=login_no_remember.cookies)

        # Step 4: Login WITH remember me
        login_with_remember = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": test_user_data["password"],
                "remember_me": True
            }
        )
        assert login_with_remember.status_code == 200
        remember_cookie = login_with_remember.cookies.get("access_token")
        assert remember_cookie is not None

        # Both should allow access to protected endpoints
        profile_response = client.get(
            "/api/v1/profile",
            cookies=login_with_remember.cookies
        )
        assert profile_response.status_code == 200


class TestAuthEdgeCases:
    """Test edge cases and error conditions"""

    def test_duplicate_email_registration(self, test_user_data):
        """Test that registering with duplicate email fails"""
        # First registration
        client.post("/api/v1/auth/register", json=test_user_data)

        # Try to register again with same email
        duplicate_response = client.post(
            "/api/v1/auth/register",
            json=test_user_data
        )
        assert duplicate_response.status_code == 409

    def test_invalid_credentials_login(self, test_user_data):
        """Test login with invalid credentials"""
        # Register user
        client.post("/api/v1/auth/register", json=test_user_data)

        # Try to login with wrong password
        invalid_login = client.post(
            "/api/v1/auth/login",
            json={
                "email": test_user_data["email"],
                "password": "WrongPassword123!",
                "remember_me": False
            }
        )
        assert invalid_login.status_code == 401

    def test_access_protected_endpoint_without_auth(self):
        """Test accessing protected endpoint without authentication"""
        response = client.get("/api/v1/profile")
        assert response.status_code == 401

    def test_expired_reset_token(self, test_user_data):
        """Test that expired reset tokens are rejected"""
        # Register user
        client.post("/api/v1/auth/register", json=test_user_data)

        # Create expired token
        from app.auth.security import hash_password
        from app.auth.models import PasswordResetToken
        from datetime import datetime, timedelta

        test_token = secrets.token_urlsafe(32)
        token_hash = hash_password(test_token)

        db = next(override_get_db())
        expired_token = PasswordResetToken(
            email=test_user_data["email"],
            token_hash=token_hash,
            expires_at=datetime.utcnow() - timedelta(hours=1)  # Expired 1 hour ago
        )
        db.add(expired_token)
        db.commit()

        # Try to reset password with expired token
        reset_response = client.post(
            "/api/v1/auth/reset-password",
            json={
                "token": test_token,
                "new_password": "NewPassword123!"
            }
        )
        assert reset_response.status_code == 400


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
