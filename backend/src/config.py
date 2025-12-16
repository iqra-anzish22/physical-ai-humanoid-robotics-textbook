from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    # API Keys and URLs
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    neon_database_url: Optional[str] = os.getenv("NEON_DATABASE_URL")

    # Text processing parameters
    max_text_length: int = int(os.getenv("MAX_TEXT_LENGTH", "100000"))
    default_chunk_size: int = int(os.getenv("DEFAULT_CHUNK_SIZE", "600"))
    default_overlap: int = int(os.getenv("DEFAULT_OVERLAP", "100"))

    # Session parameters
    session_timeout: int = int(os.getenv("SESSION_TIMEOUT", "30"))  # in minutes

    # Qdrant parameters
    qdrant_collection_prefix: str = "session_"
    vector_size: int = 1024  # Cohere embed-english-v3.0 returns 1024-dimensional vectors

    # Cohere parameters
    cohere_embedding_model: str = "embed-english-v3.0"
    cohere_generation_model: str = "command-r-plus"

    class Config:
        env_file = ".env"


settings = Settings()