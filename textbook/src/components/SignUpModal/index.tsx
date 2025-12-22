import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { z } from 'zod';
import { useAuth } from '../AuthProvider';
import styles from './styles.module.css';

const signUpSchema = z.object({
  name: z.string().min(1, 'Name is required'),
  email: z.string().email('Invalid email address'),
  password: z.string().min(8, 'Password must be at least 8 characters'),
  skill_level: z.enum(['beginner', 'intermediate', 'advanced']),
  learning_goals: z.string().optional(),
  prior_experience: z.string().optional(),
});

type SignUpFormData = z.infer<typeof signUpSchema>;

interface SignUpModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function SignUpModal({ isOpen, onClose }: SignUpModalProps) {
  const { register: registerUser } = useAuth();
  const [error, setError] = useState<string | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);

  const {
    register,
    handleSubmit,
    formState: { errors },
    reset,
  } = useForm<SignUpFormData>({
    resolver: zodResolver(signUpSchema),
    defaultValues: {
      skill_level: 'beginner',
    },
  });

  const onSubmit = async (data: SignUpFormData) => {
    setError(null);
    setIsSubmitting(true);

    try {
      await registerUser(data);
      reset();
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Registration failed. Please try again.');
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
      aria-labelledby="signup-modal-title"
    >
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <h2 id="signup-modal-title">Create Your Account</h2>
          <button className={styles.closeButton} onClick={onClose} aria-label="Close signup modal">
            ×
          </button>
        </div>

        <form onSubmit={handleSubmit(onSubmit)} className={styles.form} aria-label="Sign up form">
          {error && (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="name">Full Name</label>
            <input
              id="name"
              type="text"
              {...register('name')}
              placeholder="Enter your full name"
              aria-invalid={errors.name ? 'true' : 'false'}
            />
            {errors.name && <span className={styles.errorMessage}>{errors.name.message}</span>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="email">Email Address</label>
            <input
              id="email"
              type="email"
              {...register('email')}
              placeholder="your.email@example.com"
              aria-invalid={errors.email ? 'true' : 'false'}
            />
            {errors.email && <span className={styles.errorMessage}>{errors.email.message}</span>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              {...register('password')}
              placeholder="At least 8 characters"
              aria-invalid={errors.password ? 'true' : 'false'}
            />
            {errors.password && <span className={styles.errorMessage}>{errors.password.message}</span>}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="skill_level">Skill Level</label>
            <select id="skill_level" {...register('skill_level')}>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="learning_goals">Learning Goals (Optional)</label>
            <textarea
              id="learning_goals"
              {...register('learning_goals')}
              placeholder="What do you want to achieve?"
              rows={3}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="prior_experience">Prior Experience (Optional)</label>
            <textarea
              id="prior_experience"
              {...register('prior_experience')}
              placeholder="Describe your background in robotics"
              rows={3}
            />
          </div>

          <button type="submit" className={styles.submitButton} disabled={isSubmitting}>
            {isSubmitting ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>
      </div>
    </div>
  );
}
