"""
Text processing utilities for code detection and preservation.

Provides functions to detect, preserve, and restore code blocks and technical
terms during translation and summarization operations.
"""

import re
from typing import Dict, List, Tuple


# Regex patterns for code block detection
CODE_BLOCK_PATTERNS = [
    r'```[\s\S]*?```',  # Markdown code blocks (fenced)
    r'`[^`]+`',  # Inline code
    r'<code>[\s\S]*?</code>',  # HTML code tags
    r'<pre>[\s\S]*?</pre>',  # HTML pre tags
]

# Technical terms and keywords to preserve (common programming terms)
TECHNICAL_TERMS = [
    'API', 'HTTP', 'HTTPS', 'REST', 'GraphQL', 'JSON', 'XML', 'SQL', 'NoSQL',
    'async', 'await', 'Promise', 'callback', 'function', 'const', 'let', 'var',
    'class', 'interface', 'type', 'enum', 'struct', 'def', 'import', 'export',
    'React', 'Vue', 'Angular', 'Node.js', 'Python', 'JavaScript', 'TypeScript',
    'FastAPI', 'Django', 'Flask', 'Express', 'database', 'query', 'schema',
]


def detect_code_blocks(text: str) -> List[Dict[str, any]]:
    """
    Detect code blocks in text using regex patterns.

    Args:
        text: Input text to scan for code blocks

    Returns:
        List of dictionaries with code block information:
        - match: The matched code block text
        - start: Starting index in text
        - end: Ending index in text
        - pattern: Pattern type that matched
    """
    code_blocks = []

    for pattern in CODE_BLOCK_PATTERNS:
        for match in re.finditer(pattern, text):
            code_blocks.append({
                'match': match.group(0),
                'start': match.start(),
                'end': match.end(),
                'pattern': pattern
            })

    # Sort by start position to maintain order
    code_blocks.sort(key=lambda x: x['start'])

    return code_blocks


def preserve_code_blocks(text: str) -> Tuple[str, Dict[str, str]]:
    """
    Replace code blocks with placeholders for preservation during translation.

    Args:
        text: Input text containing code blocks

    Returns:
        Tuple of (processed_text, placeholder_map):
        - processed_text: Text with code blocks replaced by placeholders
        - placeholder_map: Mapping of placeholders to original code blocks
    """
    code_blocks = detect_code_blocks(text)
    placeholder_map = {}
    processed_text = text

    # Replace code blocks with placeholders (in reverse order to maintain indices)
    for idx, block in enumerate(reversed(code_blocks)):
        placeholder = f"__CODE_BLOCK_{len(code_blocks) - idx - 1}__"
        placeholder_map[placeholder] = block['match']
        processed_text = (
            processed_text[:block['start']] +
            placeholder +
            processed_text[block['end']:]
        )

    return processed_text, placeholder_map


def restore_code_blocks(text: str, placeholder_map: Dict[str, str]) -> str:
    """
    Restore original code blocks from placeholders.

    Args:
        text: Text with placeholders
        placeholder_map: Mapping of placeholders to original code blocks

    Returns:
        Text with code blocks restored
    """
    restored_text = text

    for placeholder, original_code in placeholder_map.items():
        restored_text = restored_text.replace(placeholder, original_code)

    return restored_text


def detect_technical_terms(text: str) -> List[str]:
    """
    Detect technical terms and keywords in text.

    Args:
        text: Input text to scan for technical terms

    Returns:
        List of detected technical terms
    """
    detected_terms = []

    for term in TECHNICAL_TERMS:
        # Case-insensitive word boundary search
        pattern = r'\b' + re.escape(term) + r'\b'
        if re.search(pattern, text, re.IGNORECASE):
            detected_terms.append(term)

    return detected_terms


def count_words(text: str) -> int:
    """
    Count words in text (simple whitespace-based counting).

    Args:
        text: Input text

    Returns:
        Word count
    """
    return len(text.split())


def count_preserved_elements(placeholder_map: Dict[str, str]) -> Dict[str, int]:
    """
    Count types of preserved elements (code blocks, inline code, etc.).

    Args:
        placeholder_map: Mapping of placeholders to original code blocks

    Returns:
        Dictionary with counts by element type
    """
    counts = {
        'code_blocks': 0,  # ```...```
        'inline_code': 0,  # `...`
        'html_code': 0,    # <code>...</code>
        'html_pre': 0,     # <pre>...</pre>
    }

    for code in placeholder_map.values():
        if code.startswith('```'):
            counts['code_blocks'] += 1
        elif code.startswith('`') and code.endswith('`') and len(code) < 100:
            counts['inline_code'] += 1
        elif code.startswith('<code>'):
            counts['html_code'] += 1
        elif code.startswith('<pre>'):
            counts['html_pre'] += 1

    return counts


def extract_urls(text: str) -> List[str]:
    """
    Extract URLs from text for preservation during translation.

    Args:
        text: Input text

    Returns:
        List of detected URLs
    """
    url_pattern = r'https?://[^\s<>"{}|\\^`\[\]]+'
    urls = re.findall(url_pattern, text)
    return urls


def clean_whitespace(text: str) -> str:
    """
    Clean excessive whitespace while preserving paragraph structure.

    Args:
        text: Input text

    Returns:
        Cleaned text
    """
    # Replace multiple spaces with single space
    text = re.sub(r' +', ' ', text)

    # Replace multiple newlines with double newline (paragraph break)
    text = re.sub(r'\n{3,}', '\n\n', text)

    # Strip leading/trailing whitespace
    text = text.strip()

    return text


def truncate_text(text: str, max_words: int = 5000) -> str:
    """
    Truncate text to maximum word count.

    Args:
        text: Input text
        max_words: Maximum number of words (default: 5000)

    Returns:
        Truncated text
    """
    words = text.split()
    if len(words) <= max_words:
        return text

    truncated_words = words[:max_words]
    return ' '.join(truncated_words) + '...'
