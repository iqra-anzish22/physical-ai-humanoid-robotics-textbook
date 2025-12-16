import re
from typing import Union, List, Dict, Any
import html
import urllib.parse


class InputSanitizer:
    """Utility class for sanitizing user inputs to prevent injection attacks."""

    @staticmethod
    def sanitize_text(text: str) -> str:
        """
        Sanitize text input to prevent injection attacks.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        if not isinstance(text, str):
            return ""

        # Remove control characters except common whitespace
        sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', text)

        # Escape HTML characters to prevent XSS
        sanitized = html.escape(sanitized)

        # Remove potential SQL injection patterns (basic)
        sql_patterns = [
            r"(?i)(union\s+select)",
            r"(?i)(drop\s+table)",
            r"(?i)(delete\s+from)",
            r"(?i)(insert\s+into)",
            r"(?i)(update\s+\w+\s+set)",
            r"(?i)(exec\s*\()",
            r"(?i)(execute\s*\()",
            r"(?i)(sp_)",
            r"(?i)(xp_)",
            r"(?i)(0x[0-9a-fA-F]+)",  # Hex literals
        ]

        for pattern in sql_patterns:
            sanitized = re.sub(pattern, '', sanitized)

        # Remove potential command injection patterns
        cmd_patterns = [
            r"[;&|`$()]",  # Shell metacharacters
            r"(?i)(cmd\.exe)",
            r"(?i)(powershell)",
            r"(?i)(bash)",
            r"(?i)(sh\s+)",
        ]

        for pattern in cmd_patterns:
            sanitized = re.sub(pattern, '', sanitized)

        # Limit length to prevent buffer overflow
        if len(sanitized) > 100000:  # 100KB limit
            sanitized = sanitized[:100000]

        return sanitized.strip()

    @staticmethod
    def sanitize_session_id(session_id: str) -> str:
        """
        Sanitize session ID to prevent path traversal and other attacks.

        Args:
            session_id: Session ID to sanitize

        Returns:
            Sanitized session ID
        """
        if not isinstance(session_id, str):
            return ""

        # Only allow alphanumeric characters, hyphens, and underscores
        sanitized = re.sub(r'[^a-zA-Z0-9\-_]', '', session_id)

        # Ensure reasonable length
        if len(sanitized) > 100:
            sanitized = sanitized[:100]

        return sanitized

    @staticmethod
    def sanitize_json_input(data: Union[Dict[str, Any], List[Any]]) -> Union[Dict[str, Any], List[Any]]:
        """
        Recursively sanitize JSON input to prevent injection attacks.

        Args:
            data: JSON data to sanitize

        Returns:
            Sanitized JSON data
        """
        if isinstance(data, dict):
            sanitized_dict = {}
            for key, value in data.items():
                # Sanitize keys as well to prevent injection through object keys
                sanitized_key = InputSanitizer.sanitize_text(str(key))
                sanitized_dict[sanitized_key] = InputSanitizer.sanitize_json_input(value)
            return sanitized_dict
        elif isinstance(data, list):
            return [InputSanitizer.sanitize_json_input(item) for item in data]
        elif isinstance(data, str):
            return InputSanitizer.sanitize_text(data)
        else:
            return data

    @staticmethod
    def validate_url(url: str) -> bool:
        """
        Validate URL format to prevent SSRF and other attacks.

        Args:
            url: URL to validate

        Returns:
            True if valid, False otherwise
        """
        if not isinstance(url, str):
            return False

        try:
            parsed = urllib.parse.urlparse(url)
            # Basic validation: must have scheme and netloc
            if not parsed.scheme or not parsed.netloc:
                return False

            # Check for potentially dangerous schemes
            dangerous_schemes = ['file', 'ftp', 'gopher']
            if parsed.scheme.lower() in dangerous_schemes:
                return False

            # Basic format check
            if not re.match(r'^https?://[^\s/$.?#].[^\s]*$', url):
                return False

            return True
        except Exception:
            return False