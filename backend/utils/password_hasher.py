"""
Password hashing utilities using bcrypt (T009).

This module provides secure password hashing and verification using bcrypt
with a cost factor of 12 for OWASP-compliant security.
"""

import bcrypt
import logging

logger = logging.getLogger(__name__)

# bcrypt cost factor (number of rounds = 2^12 = 4096 rounds)
BCRYPT_COST_FACTOR = 12


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt with cost factor 12.

    Args:
        password: Plain text password to hash

    Returns:
        str: bcrypt hashed password (UTF-8 decoded string)

    Raises:
        ValueError: If password is empty or invalid
    """
    if not password or not isinstance(password, str):
        raise ValueError("Password must be a non-empty string")

    # bcrypt expects bytes, so encode password to UTF-8
    password_bytes = password.encode('utf-8')

    # Generate salt with cost factor and hash password
    salt = bcrypt.gensalt(rounds=BCRYPT_COST_FACTOR)
    hashed = bcrypt.hashpw(password_bytes, salt)

    # Return hash as UTF-8 decoded string for database storage
    return hashed.decode('utf-8')


def verify_password(password: str, password_hash: str) -> bool:
    """
    Verify a password against a bcrypt hash.

    Args:
        password: Plain text password to verify
        password_hash: bcrypt hashed password from database

    Returns:
        bool: True if password matches hash, False otherwise
    """
    if not password or not password_hash:
        return False

    try:
        # Encode both password and hash to bytes
        password_bytes = password.encode('utf-8')
        hash_bytes = password_hash.encode('utf-8')

        # Verify password using bcrypt
        return bcrypt.checkpw(password_bytes, hash_bytes)
    except Exception as e:
        logger.error(f"Error verifying password: {e}")
        return False
