import React, { useState, useEffect } from 'react';
import { useLocation, useHistory } from '@docusaurus/router';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import axios from 'axios';
import Layout from '@theme/Layout';
import styles from './reset-password.module.css';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

const resetPasswordSchema = z.object({
  password: z.string().min(8, 'Password must be at least 8 characters'),
  confirmPassword: z.string().min(8, 'Password must be at least 8 characters'),
}).refine((data) => data.password === data.confirmPassword, {
  message: "Passwords don't match",
  path: ['confirmPassword'],
});

type ResetPasswordFormData = z.infer<typeof resetPasswordSchema>;

export default function ResetPasswordPage() {
  const location = useLocation();
  const history = useHistory();
  const [token, setToken] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const {
    register,
    handleSubmit,
    formState: { errors },
  } = useForm<ResetPasswordFormData>({
    resolver: zodResolver(resetPasswordSchema),
  });

  useEffect(() => {
    // Extract token from URL query parameter
    const params = new URLSearchParams(location.search);
    const tokenParam = params.get('token');

    if (!tokenParam) {
      setError('Invalid reset link. Please request a new password reset.');
    } else {
      setToken(tokenParam);
    }
  }, [location]);

  const onSubmit = async (data: ResetPasswordFormData) => {
    if (!token) {
      setError('Invalid reset token. Please request a new password reset.');
      return;
    }

    setError(null);
    setSuccess(null);
    setIsSubmitting(true);

    try {
      await axios.post(
        `${API_BASE_URL}/api/v1/auth/reset-password`,
        {
          token: token,
          new_password: data.password,
        },
        { withCredentials: true }
      );

      setSuccess('Password reset successful! Redirecting to login...');

      // Redirect to home page after 3 seconds
      setTimeout(() => {
        history.push('/');
      }, 3000);
    } catch (err: any) {
      if (err.response?.status === 400) {
        setError('Invalid or expired reset token. Please request a new password reset.');
      } else {
        setError(
          err.response?.data?.detail ||
          'Failed to reset password. Please try again.'
        );
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <Layout
      title="Reset Password"
      description="Reset your password"
    >
      <div className={styles.container}>
        <div className={styles.card}>
          <h1 className={styles.title}>Reset Your Password</h1>

          {!token && error ? (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          ) : success ? (
            <div className={styles.successAlert} role="alert">
              {success}
            </div>
          ) : (
            <>
              {error && (
                <div className={styles.errorAlert} role="alert">
                  {error}
                </div>
              )}

              <p className={styles.description}>
                Enter your new password below. Make sure it's at least 8 characters long.
              </p>

              <form onSubmit={handleSubmit(onSubmit)} className={styles.form}>
                <div className={styles.formGroup}>
                  <label htmlFor="password">New Password</label>
                  <input
                    id="password"
                    type="password"
                    {...register('password')}
                    placeholder="At least 8 characters"
                    aria-invalid={errors.password ? 'true' : 'false'}
                    disabled={isSubmitting}
                  />
                  {errors.password && (
                    <span className={styles.errorMessage}>{errors.password.message}</span>
                  )}
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="confirmPassword">Confirm New Password</label>
                  <input
                    id="confirmPassword"
                    type="password"
                    {...register('confirmPassword')}
                    placeholder="Re-enter your password"
                    aria-invalid={errors.confirmPassword ? 'true' : 'false'}
                    disabled={isSubmitting}
                  />
                  {errors.confirmPassword && (
                    <span className={styles.errorMessage}>
                      {errors.confirmPassword.message}
                    </span>
                  )}
                </div>

                <button
                  type="submit"
                  className={styles.submitButton}
                  disabled={isSubmitting || !token}
                >
                  {isSubmitting ? 'Resetting Password...' : 'Reset Password'}
                </button>
              </form>
            </>
          )}

          <div className={styles.footer}>
            <a href="/" className={styles.backLink}>
              ← Back to Home
            </a>
          </div>
        </div>
      </div>
    </Layout>
  );
}
