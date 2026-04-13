def dispatch(*_types, **_kwargs):
    def decorator(func):
        return func
    return decorator
