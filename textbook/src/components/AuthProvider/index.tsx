import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import axios from 'axios';

interface User {
  id: number;
  email: string;
  is_active: boolean;
  created_at: string;
  last_login_at: string | null;
  profile: {
    id: number;
    user_id: number;
    name: string;
    skill_level: 'beginner' | 'intermediate' | 'advanced';
    learning_goals: string | null;
    prior_experience: string | null;
    created_at: string;
    updated_at: string;
  } | null;
}

interface AuthContextType {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string, rememberMe?: boolean) => Promise<void>;
  register: (userData: RegisterData) => Promise<void>;
  logout: () => void;
}

interface RegisterData {
  email: string;
  password: string;
  name: string;
  skill_level?: 'beginner' | 'intermediate' | 'advanced';
  learning_goals?: string;
  prior_experience?: string;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api.example.com'
  : 'http://localhost:8000';

// Configure axios to send cookies with requests
axios.defaults.withCredentials = true;

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Try to fetch current user using httpOnly cookie
    // If cookie exists and is valid, user will be authenticated
    fetchCurrentUser();
  }, []);

  const fetchCurrentUser = async () => {
    try {
      // Cookie will be automatically sent with this request
      const response = await axios.get(`${API_BASE_URL}/api/v1/auth/me`);
      setUser(response.data);
      // Token is in cookie, but we set a placeholder for UI state
      setToken('cookie-auth');
    } catch (error) {
      // No valid cookie or authentication failed
      console.error('Failed to fetch current user:', error);
      setUser(null);
      setToken(null);
    } finally {
      setIsLoading(false);
    }
  };

  const login = async (email: string, password: string, rememberMe = false) => {
    try {
      // Cookie will be automatically set by backend
      const response = await axios.post(`${API_BASE_URL}/api/v1/auth/login`, {
        email,
        password,
        remember_me: rememberMe,
      });

      const { user: userData } = response.data;
      setToken('cookie-auth'); // Token is in httpOnly cookie
      setUser(userData);
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(error.response?.data?.detail || 'Login failed');
      }
      throw error;
    }
  };

  const register = async (userData: RegisterData) => {
    try {
      // Cookie will be automatically set by backend
      const response = await axios.post(`${API_BASE_URL}/api/v1/auth/register`, userData);

      const { user: newUser } = response.data;
      setToken('cookie-auth'); // Token is in httpOnly cookie
      setUser(newUser);
    } catch (error) {
      if (axios.isAxiosError(error)) {
        throw new Error(error.response?.data?.detail || 'Registration failed');
      }
      throw error;
    }
  };

  const logout = async () => {
    try {
      // Call backend to clear cookie
      await axios.post(`${API_BASE_URL}/api/v1/auth/logout`);
    } catch (error) {
      console.error('Logout request failed:', error);
    } finally {
      // Clear local state regardless of API call result
      setUser(null);
      setToken(null);
    }
  };

  const value: AuthContextType = {
    user,
    token,
    isAuthenticated: !!user,
    isLoading,
    login,
    register,
    logout,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
