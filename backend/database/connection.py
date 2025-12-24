"""
Neon PostgreSQL database connection management using asyncpg.

This module provides connection pooling for the Neon PostgreSQL database,
supporting user profiles, interests, and personalization features.
"""

import os
import asyncpg
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def get_db_pool() -> asyncpg.Pool:
    """
    Get or create the database connection pool.

    Returns:
        asyncpg.Pool: Database connection pool

    Raises:
        ValueError: If NEON_DATABASE_URL is not set
        asyncpg.PostgresError: If connection fails
    """
    global _pool

    if _pool is None:
        database_url = os.getenv("NEON_DATABASE_URL")
        if not database_url:
            raise ValueError(
                "NEON_DATABASE_URL environment variable is not set. "
                "Please configure Neon PostgreSQL connection string in .env file."
            )

        try:
            # Create connection pool with sensible defaults
            _pool = await asyncpg.create_pool(
                database_url,
                min_size=2,  # Minimum connections in pool
                max_size=10,  # Maximum connections in pool
                command_timeout=60,  # Command timeout in seconds
                timeout=30,  # Connection timeout
            )
            logger.info("Database connection pool created successfully")
        except Exception as e:
            logger.error(f"Failed to create database pool: {e}")
            raise

    return _pool


async def close_db_pool() -> None:
    """
    Close the database connection pool.

    Should be called when the application shuts down.
    """
    global _pool

    if _pool is not None:
        await _pool.close()
        _pool = None
        logger.info("Database connection pool closed")


async def execute_query(query: str, *args) -> list:
    """
    Execute a SELECT query and return results.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        list: Query results as list of Record objects
    """
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        return await conn.fetch(query, *args)


async def execute_command(query: str, *args) -> str:
    """
    Execute an INSERT/UPDATE/DELETE command.

    Args:
        query: SQL command string
        *args: Command parameters

    Returns:
        str: Command status (e.g., "INSERT 0 1")
    """
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        return await conn.execute(query, *args)
