-- Migration 002: Seed interest categories
-- Date: 2025-12-18
-- Feature: urdu-translation-personalization
--
-- This migration seeds the interest_categories table with 8 predefined categories
-- that users can select from during sign-up or in their profile settings.
--
-- These categories are used for personalizing chatbot responses with relevant
-- analogies and examples based on user interests.

BEGIN;

-- Insert 8 predefined interest categories
-- Using ON CONFLICT DO NOTHING to make migration idempotent
INSERT INTO interest_categories (name, slug, description) VALUES
    ('Robotics', 'robotics', 'General robotics and automation'),
    ('AI/ML', 'ai_ml', 'Artificial Intelligence and Machine Learning'),
    ('Software Engineering', 'software_engineering', 'Software development and architecture'),
    ('Mechanical Engineering', 'mechanical_engineering', 'Mechanical systems and design'),
    ('Electrical Engineering', 'electrical_engineering', 'Electronics and circuits'),
    ('Mathematics', 'mathematics', 'Mathematical foundations and theory'),
    ('Physics', 'physics', 'Physics principles and applications'),
    ('Computer Vision', 'computer_vision', 'Vision systems and image processing')
ON CONFLICT (slug) DO NOTHING;

COMMIT;

-- Verification query:
-- SELECT * FROM interest_categories ORDER BY id;
--
-- Expected result: 8 rows with the categories listed above

-- Rollback instructions:
-- DELETE FROM interest_categories WHERE slug IN (
--     'robotics', 'ai_ml', 'software_engineering', 'mechanical_engineering',
--     'electrical_engineering', 'mathematics', 'physics', 'computer_vision'
-- );
