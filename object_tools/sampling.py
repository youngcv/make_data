import random


def sample_stratified_range(value_range, num_samples):
    """Split a range into equal bins and sample once from each bin."""
    low, high = value_range
    if num_samples <= 0:
        return []
    if low == high:
        return [low] * num_samples

    step = (high - low) / num_samples
    return [
        random.uniform(
            low + idx * step,
            high if idx == num_samples - 1 else low + (idx + 1) * step,
        )
        for idx in range(num_samples)
    ]


def sample_shuffled_axes(axis_ranges, num_samples):
    """Return one independently shuffled stratified sample list per axis."""
    return [
        sample_shuffled_range(axis_range, num_samples)
        for axis_range in axis_ranges
    ]


def sample_shuffled_range(value_range, num_samples):
    samples = sample_stratified_range(value_range, num_samples)
    random.shuffle(samples)
    return samples


def shuffled_configs(configs):
    random.shuffle(configs)
    return configs
