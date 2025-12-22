import React, { useState, useEffect } from 'react';
import axios from 'axios';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthProvider';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';

const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-api.example.com'
  : 'http://localhost:8000';

const profileSchema = z.object({
  name: z.string().min(1, 'Name is required'),
  skill_level: z.enum(['beginner', 'intermediate', 'advanced']),
  learning_goals: z.string().optional(),
  prior_experience: z.string().optional(),
});

type ProfileFormData = z.infer<typeof profileSchema>;

export default function ProfilePage() {
  const { user, isAuthenticated, isLoading } = useAuth();
  const [saving, setSaving] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  const {
    register,
    handleSubmit,
    formState: { errors },
    reset,
  } = useForm<ProfileFormData>({
    resolver: zodResolver(profileSchema),
  });

  useEffect(() => {
    if (user?.profile) {
      reset({
        name: user.profile.name,
        skill_level: user.profile.skill_level,
        learning_goals: user.profile.learning_goals || '',
        prior_experience: user.profile.prior_experience || '',
      });
    }
  }, [user, reset]);

  const onSubmit = async (data: ProfileFormData) => {
    setSaving(true);
    setMessage(null);

    try {
      await axios.put(`${API_BASE_URL}/api/v1/profile`, data);
      setMessage({ type: 'success', text: 'Profile updated successfully!' });
    } catch (error) {
      setMessage({ type: 'error', text: 'Failed to update profile. Please try again.' });
    } finally {
      setSaving(false);
    }
  };

  if (isLoading) {
    return (
      <Layout title="My Profile" description="Manage your learning profile">
        <div style={{ padding: '2rem', textAlign: 'center' }}>Loading...</div>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="My Profile" description="Manage your learning profile">
        <div style={{ padding: '2rem', textAlign: 'center' }}>
          <h1>Authentication Required</h1>
          <p>Please sign in to view your profile.</p>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="My Profile" description="Manage your learning profile">
      <div style={{ maxWidth: '600px', margin: '2rem auto', padding: '0 1rem' }}>
        <h1>My Learning Profile</h1>

        {message && (
          <div
            style={{
              padding: '1rem',
              marginBottom: '1rem',
              borderRadius: '4px',
              backgroundColor: message.type === 'success' ? '#d4edda' : '#f8d7da',
              color: message.type === 'success' ? '#155724' : '#721c24',
            }}
          >
            {message.text}
          </div>
        )}

        <form onSubmit={handleSubmit(onSubmit)}>
          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="name" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
              Name
            </label>
            <input
              id="name"
              type="text"
              {...register('name')}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
              }}
            />
            {errors.name && <span style={{ color: 'red', fontSize: '0.875rem' }}>{errors.name.message}</span>}
          </div>

          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="skill_level" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
              Skill Level
            </label>
            <select
              id="skill_level"
              {...register('skill_level')}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
              }}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="learning_goals" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
              Learning Goals
            </label>
            <textarea
              id="learning_goals"
              {...register('learning_goals')}
              rows={3}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
              }}
            />
          </div>

          <div style={{ marginBottom: '1rem' }}>
            <label htmlFor="prior_experience" style={{ display: 'block', marginBottom: '0.5rem', fontWeight: 'bold' }}>
              Prior Experience
            </label>
            <textarea
              id="prior_experience"
              {...register('prior_experience')}
              rows={3}
              style={{
                width: '100%',
                padding: '0.5rem',
                border: '1px solid #ccc',
                borderRadius: '4px',
              }}
            />
          </div>

          <button
            type="submit"
            disabled={saving}
            style={{
              padding: '0.75rem 1.5rem',
              backgroundColor: '#007bff',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: saving ? 'not-allowed' : 'pointer',
              fontSize: '1rem',
            }}
          >
            {saving ? 'Saving...' : 'Save Profile'}
          </button>
        </form>
      </div>
    </Layout>
  );
}
