import inspect
from transformers.models.opt import modeling_opt

# List all classes
classes = [name for name, obj in inspect.getmembers(modeling_opt) if inspect.isclass(obj)]
print("Classes:", classes)

# List all functions
functions = [name for name, obj in inspect.getmembers(modeling_opt) if inspect.isfunction(obj)]
print("Functions:", functions)
