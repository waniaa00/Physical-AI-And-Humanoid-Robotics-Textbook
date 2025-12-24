"""
Base repository class implementing repository pattern for data access.

This provides a common interface for database operations using asyncpg.
"""

from typing import List, Optional, Any, Dict
import asyncpg
from database.connection import get_db_pool
import logging

logger = logging.getLogger(__name__)


class BaseRepository:
    """
    Base repository class for database operations.

    Provides common CRUD operations and query execution methods.
    Child repositories can extend this for entity-specific operations.
    """

    def __init__(self):
        """Initialize the repository."""
        self.pool: Optional[asyncpg.Pool] = None

    async def _get_pool(self) -> asyncpg.Pool:
        """
        Get database connection pool.

        Returns:
            asyncpg.Pool: Database connection pool
        """
        if self.pool is None:
            self.pool = await get_db_pool()
        return self.pool

    async def fetch_one(self, query: str, *args) -> Optional[asyncpg.Record]:
        """
        Fetch a single row from the database.

        Args:
            query: SQL query string
            *args: Query parameters

        Returns:
            Optional[Record]: Single row result or None if not found
        """
        pool = await self._get_pool()
        try:
            async with pool.acquire() as conn:
                result = await conn.fetchrow(query, *args)
                return result
        except Exception as e:
            logger.error(f"Error fetching one: {e}")
            raise

    async def fetch_all(self, query: str, *args) -> List[asyncpg.Record]:
        """
        Fetch multiple rows from the database.

        Args:
            query: SQL query string
            *args: Query parameters

        Returns:
            List[Record]: List of row results (empty list if no results)
        """
        pool = await self._get_pool()
        try:
            async with pool.acquire() as conn:
                results = await conn.fetch(query, *args)
                return results
        except Exception as e:
            logger.error(f"Error fetching all: {e}")
            raise

    async def execute(self, query: str, *args) -> str:
        """
        Execute an INSERT/UPDATE/DELETE command.

        Args:
            query: SQL command string
            *args: Command parameters

        Returns:
            str: Command status (e.g., "INSERT 0 1")
        """
        pool = await self._get_pool()
        try:
            async with pool.acquire() as conn:
                result = await conn.execute(query, *args)
                return result
        except Exception as e:
            logger.error(f"Error executing command: {e}")
            raise

    async def execute_many(self, query: str, args_list: List[tuple]) -> None:
        """
        Execute a command multiple times with different parameters.

        Args:
            query: SQL command string
            args_list: List of parameter tuples
        """
        pool = await self._get_pool()
        try:
            async with pool.acquire() as conn:
                await conn.executemany(query, args_list)
        except Exception as e:
            logger.error(f"Error executing many: {e}")
            raise

    async def fetch_val(self, query: str, *args) -> Any:
        """
        Fetch a single value from the database.

        Args:
            query: SQL query string
            *args: Query parameters

        Returns:
            Any: Single value result
        """
        pool = await self._get_pool()
        try:
            async with pool.acquire() as conn:
                result = await conn.fetchval(query, *args)
                return result
        except Exception as e:
            logger.error(f"Error fetching value: {e}")
            raise

    def dict_from_record(self, record: Optional[asyncpg.Record]) -> Optional[Dict[str, Any]]:
        """
        Convert an asyncpg Record to a dictionary.

        Args:
            record: asyncpg Record object

        Returns:
            Optional[Dict]: Dictionary representation or None if record is None
        """
        if record is None:
            return None
        return dict(record)

    def dicts_from_records(self, records: List[asyncpg.Record]) -> List[Dict[str, Any]]:
        """
        Convert a list of asyncpg Records to a list of dictionaries.

        Args:
            records: List of asyncpg Record objects

        Returns:
            List[Dict]: List of dictionary representations
        """
        return [dict(record) for record in records]
