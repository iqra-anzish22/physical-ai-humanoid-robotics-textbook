import time
import logging
from typing import Callable, Any
from functools import wraps
from asyncio import iscoroutinefunction
from ..config import settings


class PerformanceMonitor:
    def __init__(self):
        self.threshold = 5.0  # 5 seconds threshold as per requirements

    def measure_time(self, func: Callable) -> Callable:
        """
        Decorator to measure execution time of a function.
        """
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = await func(*args, **kwargs)
                execution_time = time.time() - start_time
                self._log_performance(func.__name__, execution_time)
                return result
            except Exception as e:
                execution_time = time.time() - start_time
                self._log_performance(func.__name__, execution_time, error=str(e))
                raise

        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time
                self._log_performance(func.__name__, execution_time)
                return result
            except Exception as e:
                execution_time = time.time() - start_time
                self._log_performance(func.__name__, execution_time, error=str(e))
                raise

        # Return the appropriate wrapper based on function type
        if iscoroutinefunction(func):
            return async_wrapper
        else:
            return sync_wrapper

    def _log_performance(self, func_name: str, execution_time: float, error: str = None):
        """
        Log performance metrics.
        """
        status = "SLOW" if execution_time > self.threshold else "OK"
        log_msg = f"PERFORMANCE [{status}]: {func_name} took {execution_time:.3f}s"

        if error:
            log_msg += f" (ERROR: {error})"

        if execution_time > self.threshold:
            logging.warning(log_msg)
        else:
            logging.info(log_msg)

    def validate_response_time(self, execution_time: float) -> bool:
        """
        Validate if execution time is within acceptable limits.

        Args:
            execution_time: Time taken to execute an operation in seconds

        Returns:
            True if within limits, False otherwise
        """
        return execution_time <= self.threshold

    def get_performance_stats(self) -> dict:
        """
        Get overall performance statistics.
        """
        # This would be expanded in a real implementation to track historical data
        return {
            "response_time_threshold": self.threshold,
            "current_threshold": self.threshold
        }


# Global performance monitor instance
performance_monitor = PerformanceMonitor()