from abc import ABC
from typing import Any, Dict, Type, TypeVar

T = TypeVar("T")


class Api(ABC):
    pass


class ApiRegistry:
    _instances: Dict[Type, object] = {}

    @classmethod
    def register(cls, api_class: Type[T], *args, **kwargs) -> None:
        cls._instances[api_class] = api_class(*args, **kwargs)

    @classmethod
    def get(cls, api_class: Type[T]) -> T:
        return cls._instances[api_class]
