-- Migration 001: Create user profiles and interests tables
-- Date: 2025-12-18
-- Feature: urdu-translation-personalization
--
-- This migration creates the core database schema for user personalization:
-- - user_profiles: Stores user background and language preferences
-- - interest_categories: Predefined interest categories (seeded separately in 002)
-- - user_interests: Junction table linking users to their selected interests
--
-- NOTE: This migration assumes better-auth users table exists.
-- If not using better-auth, remove the foreign key constraint on user_profiles.

BEGIN;

-- Create user_profiles table
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id UUID PRIMARY KEY,
    background VARCHAR(20) NOT NULL CHECK (background IN ('student', 'professional')),
    language_preference VARCHAR(10) DEFAULT 'en',
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Create interest_categories table
CREATE TABLE IF NOT EXISTS interest_categories (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) UNIQUE NOT NULL,
    slug VARCHAR(100) UNIQUE NOT NULL,
    description TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Create user_interests junction table
CREATE TABLE IF NOT EXISTS user_interests (
    id SERIAL PRIMARY KEY,
    user_id UUID NOT NULL,
    interest_id INTEGER NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_user_interests_user_id FOREIGN KEY (user_id) REFERENCES user_profiles(user_id) ON DELETE CASCADE,
    CONSTRAINT fk_user_interests_interest_id FOREIGN KEY (interest_id) REFERENCES interest_categories(id) ON DELETE CASCADE,
    CONSTRAINT uq_user_interests_unique UNIQUE (user_id, interest_id)
);

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX IF NOT EXISTS idx_user_interests_user_id ON user_interests(user_id);
CREATE INDEX IF NOT EXISTS idx_user_interests_interest_id ON user_interests(interest_id);

COMMIT;

-- Rollback instructions:
-- DROP TABLE IF EXISTS user_interests CASCADE;
-- DROP TABLE IF EXISTS interest_categories CASCADE;
-- DROP TABLE IF EXISTS user_profiles CASCADE;
