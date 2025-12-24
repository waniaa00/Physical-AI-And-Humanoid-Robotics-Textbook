-- Migration 006: Create authentication tables for Better Auth Integration
-- Created: 2025-12-18
-- Phase 2: Foundational (T014)

-- Create users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_sign_in_at TIMESTAMP WITH TIME ZONE,
    account_status VARCHAR(20) DEFAULT 'active' CHECK (account_status IN ('active', 'suspended', 'deleted'))
);

-- Create index on email for fast lookups
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Create index on account_status for filtering
CREATE INDEX IF NOT EXISTS idx_users_account_status ON users(account_status);

-- Create index on created_at for analytics
CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);

-- Create sessions table
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    ip_address VARCHAR(45),  -- IPv6 compatible (max 45 chars)
    user_agent TEXT
);

-- Create index on user_id for session lookups
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);

-- Create index on token_hash for revocation checks
CREATE INDEX IF NOT EXISTS idx_sessions_token_hash ON sessions(token_hash);

-- Create index on expires_at for cleanup queries
CREATE INDEX IF NOT EXISTS idx_sessions_expires_at ON sessions(expires_at);

-- Ensure user_interests table exists (created in Phase 5, but verify here)
-- This is a junction table linking users to interest_categories
CREATE TABLE IF NOT EXISTS user_interests (
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    interest_id INTEGER NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    PRIMARY KEY (user_id, interest_id)
);

-- Create index on user_id for fast interest retrieval
CREATE INDEX IF NOT EXISTS idx_user_interests_user_id ON user_interests(user_id);

-- Create index on interest_id for analytics
CREATE INDEX IF NOT EXISTS idx_user_interests_interest_id ON user_interests(interest_id);

-- Comments for documentation
COMMENT ON TABLE users IS 'Authenticated user accounts with email and password authentication';
COMMENT ON COLUMN users.email IS 'User email address (unique, case-insensitive)';
COMMENT ON COLUMN users.password_hash IS 'bcrypt hashed password (cost factor 12)';
COMMENT ON COLUMN users.last_sign_in_at IS 'Timestamp of last successful sign-in';
COMMENT ON COLUMN users.account_status IS 'Account status: active, suspended, or deleted';

COMMENT ON TABLE sessions IS 'Active authentication sessions with JWT tokens';
COMMENT ON COLUMN sessions.token_hash IS 'Hashed JWT for revocation lookup';
COMMENT ON COLUMN sessions.expires_at IS 'Session expiration timestamp (7 days from creation)';
COMMENT ON COLUMN sessions.ip_address IS 'Client IP address for security auditing';
COMMENT ON COLUMN sessions.user_agent IS 'Client user agent for security auditing';

COMMENT ON TABLE user_interests IS 'Junction table linking users to their selected interest categories';
COMMENT ON COLUMN user_interests.user_id IS 'Foreign key to users table';
COMMENT ON COLUMN user_interests.interest_id IS 'Interest category ID (2-5 per user)';
