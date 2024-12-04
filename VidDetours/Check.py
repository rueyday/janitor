try:
    from transformers.models.bloom.modeling_bloom import _expand_mask as _expand_mask_bloom
    print("Import successful!")
except ImportError as e:
    print(f"Import failed: {e}")
