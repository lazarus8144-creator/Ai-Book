import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import axios from 'axios';
import styles from './styles.module.css';

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

const passwordResetSchema = z.object({
  email: z.string().email('Invalid email address'),
});

type PasswordResetFormData = z.infer<typeof passwordResetSchema>;

interface PasswordResetModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function PasswordResetModal({ isOpen, onClose }: PasswordResetModalProps) {
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const {
    register,
    handleSubmit,
    formState: { errors },
    reset,
  } = useForm<PasswordResetFormData>({
    resolver: zodResolver(passwordResetSchema),
  });

  const onSubmit = async (data: PasswordResetFormData) => {
    setError(null);
    setSuccess(null);
    setIsSubmitting(true);

    try {
      const response = await axios.post(
        `${API_BASE_URL}/api/v1/auth/forgot-password`,
        data,
        { withCredentials: true }
      );

      setSuccess(response.data.message || 'Password reset link sent! Check your email.');
      reset();

      // Close modal after 3 seconds
      setTimeout(() => {
        onClose();
        setSuccess(null);
      }, 3000);
    } catch (err: any) {
      setError(
        err.response?.data?.detail ||
        'Failed to send reset email. Please try again.'
      );
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div
      className={styles.modalOverlay}
      onClick={onClose}
      role="dialog"
      aria-modal="true"
      aria-labelledby="password-reset-modal-title"
    >
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <h2 id="password-reset-modal-title">Reset Your Password</h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close password reset modal"
          >
            ×
          </button>
        </div>

        <form
          onSubmit={handleSubmit(onSubmit)}
          className={styles.form}
          aria-label="Password reset form"
        >
          {error && (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          )}

          {success && (
            <div className={styles.successAlert} role="alert">
              {success}
            </div>
          )}

          <p className={styles.description}>
            Enter your email address and we'll send you a link to reset your password.
          </p>

          <div className={styles.formGroup}>
            <label htmlFor="reset-email">Email Address</label>
            <input
              id="reset-email"
              type="email"
              {...register('email')}
              placeholder="your.email@example.com"
              aria-invalid={errors.email ? 'true' : 'false'}
              disabled={isSubmitting}
            />
            {errors.email && (
              <span className={styles.errorMessage}>{errors.email.message}</span>
            )}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isSubmitting}
          >
            {isSubmitting ? 'Sending...' : 'Send Reset Link'}
          </button>

          <button
            type="button"
            className={styles.cancelButton}
            onClick={onClose}
            disabled={isSubmitting}
          >
            Cancel
          </button>
        </form>
      </div>
    </div>
  );
}
