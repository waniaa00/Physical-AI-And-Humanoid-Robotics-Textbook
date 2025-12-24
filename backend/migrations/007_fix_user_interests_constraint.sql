-- Migration 007: Fix user_interests foreign key constraint to point to users table instead of user_profiles
-- This resolves the issue where user_interests was referencing user_profiles but should reference users
-- Created: 2025-12-20

BEGIN;

-- Drop the existing foreign key constraint on user_interests table that points to user_profiles
ALTER TABLE user_interests DROP CONSTRAINT IF EXISTS fk_user_interests_user_id;

-- Drop the interest_id constraint if it exists (to recreate properly)
ALTER TABLE user_interests DROP CONSTRAINT IF EXISTS fk_user_interests_interest_id;

-- Add the correct foreign key constraint pointing to the users table
ALTER TABLE user_interests ADD CONSTRAINT fk_user_interests_user_id
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE;

-- Add the interest_id foreign key constraint to interest_categories
ALTER TABLE user_interests ADD CONSTRAINT fk_user_interests_interest_id
    FOREIGN KEY (interest_id) REFERENCES interest_categories(id) ON DELETE CASCADE;

-- Create indexes if they don't exist
CREATE INDEX IF NOT EXISTS idx_user_interests_user_id ON user_interests(user_id);
CREATE INDEX IF NOT EXISTS idx_user_interests_interest_id ON user_interests(interest_id);

COMMIT;