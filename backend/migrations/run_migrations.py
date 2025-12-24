"""
Database migration runner script.

This script runs SQL migration files in order against the Neon PostgreSQL database.
Run this after setting up NEON_DATABASE_URL in your .env file.

Usage:
    python backend/migrations/run_migrations.py
"""

import asyncio
import asyncpg
import os
from pathlib import Path
from dotenv import load_dotenv
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Migration files directory
MIGRATIONS_DIR = Path(__file__).parent

# Migration files in order
MIGRATIONS = [
    "001_create_user_profiles_and_interests.sql",
    "002_seed_interest_categories.sql",
    "006_create_auth_tables.sql",
    "007_fix_user_interests_constraint.sql",
]


async def run_migration(conn: asyncpg.Connection, migration_file: str) -> None:
    """
    Run a single migration file.

    Args:
        conn: Database connection
        migration_file: Migration filename
    """
    migration_path = MIGRATIONS_DIR / migration_file

    if not migration_path.exists():
        logger.error(f"Migration file not found: {migration_file}")
        raise FileNotFoundError(f"Migration file not found: {migration_file}")

    logger.info(f"Running migration: {migration_file}")

    with open(migration_path, 'r', encoding='utf-8') as f:
        sql = f.read()

    try:
        await conn.execute(sql)
        logger.info(f"✓ Migration completed: {migration_file}")
    except Exception as e:
        logger.error(f"✗ Migration failed: {migration_file}")
        logger.error(f"Error: {e}")
        raise


async def verify_schema(conn: asyncpg.Connection) -> None:
    """
    Verify database schema after migrations.

    Args:
        conn: Database connection
    """
    logger.info("Verifying database schema...")

    # Check tables exist
    tables_query = """
        SELECT table_name
        FROM information_schema.tables
        WHERE table_schema = 'public'
        AND table_name IN ('user_profiles', 'interest_categories', 'user_interests', 'users', 'sessions')
        ORDER BY table_name;
    """
    tables = await conn.fetch(tables_query)
    logger.info(f"Found {len(tables)} tables:")
    for table in tables:
        logger.info(f"  - {table['table_name']}")

    # Check interest categories count
    count_query = "SELECT COUNT(*) as count FROM interest_categories;"
    result = await conn.fetchrow(count_query)
    category_count = result['count']
    logger.info(f"Interest categories seeded: {category_count}/8")

    if category_count != 8:
        logger.warning(f"Expected 8 interest categories, found {category_count}")

    # List interest categories
    categories_query = "SELECT id, name, slug FROM interest_categories ORDER BY id;"
    categories = await conn.fetch(categories_query)
    logger.info("Interest categories:")
    for cat in categories:
        logger.info(f"  {cat['id']}. {cat['name']} ({cat['slug']})")

    logger.info("✓ Schema verification complete")


async def main():
    """Main migration runner."""
    database_url = os.getenv("NEON_DATABASE_URL")

    if not database_url:
        logger.error("NEON_DATABASE_URL environment variable is not set")
        logger.error("Please configure your Neon PostgreSQL connection string in .env file")
        return

    logger.info("Connecting to Neon PostgreSQL database...")

    try:
        conn = await asyncpg.connect(database_url)
        logger.info("✓ Database connection established")

        # Run migrations
        for migration in MIGRATIONS:
            await run_migration(conn, migration)

        # Verify schema
        await verify_schema(conn)

        await conn.close()
        logger.info("✓ All migrations completed successfully")

    except asyncpg.PostgresError as e:
        logger.error(f"Database error: {e}")
        logger.error("Please check your NEON_DATABASE_URL and database connection")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
