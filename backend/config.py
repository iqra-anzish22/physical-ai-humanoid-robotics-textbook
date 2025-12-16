from pydantic_settings import BaseSettings
from typing import Optional
import logging

# Configure logging to only show variable names, not values
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    Will fail fast if required variables are missing.
    """

    # Required environment variables
    qdrant_api_key: str
    qdrant_url: str
    cohere_api_key: str

    # Optional environment variable
    neon_database_url: Optional[str] = None

    class Config:
        env_file = ".env"
        env_file_encoding = 'utf-8'

    def validate_settings(self):
        """
        Validate that all required settings are properly loaded
        """
        required_vars = [
            ('qdrant_api_key', self.qdrant_api_key),
            ('qdrant_url', self.qdrant_url),
            ('cohere_api_key', self.cohere_api_key),
        ]

        missing_vars = []
        for var_name, var_value in required_vars:
            if not var_value or var_value.strip() == "":
                missing_vars.append(var_name)

        if missing_vars:
            error_msg = f"Missing required environment variables: {', '.join(missing_vars)}"
            logger.error(error_msg)
            raise ValueError(error_msg)

        # Log which variables are loaded (names only, no values)
        logger.info("Environment variables loaded successfully:")
        logger.info("- qdrant_api_key: *** (loaded)")
        logger.info("- qdrant_url: *** (loaded)")
        logger.info("- cohere_api_key: *** (loaded)")
        if self.neon_database_url:
            logger.info("- neon_database_url: *** (loaded)")
        else:
            logger.info("- neon_database_url: not provided (optional)")


# Create a global instance of settings
settings = Settings()

# Validate settings on import
settings.validate_settings()