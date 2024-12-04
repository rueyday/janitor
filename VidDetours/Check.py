import inspect
from transformers.models.bloom import modeling_bloom

# List all classes
classes = [name for name, obj in inspect.getmembers(modeling_bloom) if inspect.isclass(obj)]
print("Classes:", classes)

# List all functions
functions = [name for name, obj in inspect.getmembers(modeling_bloom) if inspect.isfunction(obj)]
print("Functions:", functions)
