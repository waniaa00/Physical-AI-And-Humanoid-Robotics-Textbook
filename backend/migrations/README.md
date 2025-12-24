# Database Migrations

This directory contains SQL migration files for the Neon PostgreSQL database.

## Migration Files

Migrations are numbered sequentially and should be applied in order:

1. `001_create_user_profiles_and_interests.sql` - Create tables for user profiles, interest categories, and user interests
2. `002_seed_interest_categories.sql` - Seed predefined interest categories

## Running Migrations

### Manual Execution

Connect to your Neon database and execute the migration files in order:

```bash
# Using psql
psql $NEON_DATABASE_URL -f migrations/001_create_user_profiles_and_interests.sql
psql $NEON_DATABASE_URL -f migrations/002_seed_interest_categories.sql
```

### Using asyncpg (Python)

```python
import asyncpg
import os

async def run_migration(filename: str):
    conn = await asyncpg.connect(os.getenv("NEON_DATABASE_URL"))
    with open(filename, 'r') as f:
        sql = f.read()
    await conn.execute(sql)
    await conn.close()

# Run migrations
await run_migration('migrations/001_create_user_profiles_and_interests.sql')
await run_migration('migrations/002_seed_interest_categories.sql')
```

## Migration Best Practices

- Migrations should be idempotent (can be run multiple times safely)
- Use `CREATE TABLE IF NOT EXISTS` for table creation
- Use `INSERT ... ON CONFLICT DO NOTHING` for data seeding
- Include rollback instructions in comments
- Test migrations on a development database first

## Schema Verification

After running migrations, verify the schema:

```sql
-- List all tables
\dt

-- Describe user_profiles table
\d user_profiles

-- Describe interest_categories table
\d interest_categories

-- Describe user_interests table
\d user_interests

-- Check interest categories
SELECT * FROM interest_categories;
```
