"""Remove duplicate objects produced by different YOLO models."""


def cross_model_keep_indices(candidates, iou_threshold=0.6):
    """Return retained indices in input order.

    Each candidate is ``(model_index, (x1, y1, x2, y2))`` in image pixels.
    Models later in yolo_model_paths take priority over earlier models when
    bounding-box IoU reaches the threshold, regardless of predicted class.
    Detections from the same model are left to that model's own NMS.
    """
    if not 0.0 < iou_threshold <= 1.0:
        raise ValueError('iou_threshold must be in (0, 1]')

    kept = []
    for index in sorted(range(len(candidates)),
                        key=lambda i: candidates[i][0], reverse=True):
        model_index, box = candidates[index]
        duplicate = False
        for kept_index in kept:
            other_model_index, other = candidates[kept_index]
            if model_index == other_model_index:
                continue
            intersection = (
                max(0.0, min(box[2], other[2]) - max(box[0], other[0]))
                * max(0.0, min(box[3], other[3]) - max(box[1], other[1]))
            )
            area = max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])
            other_area = (max(0.0, other[2] - other[0])
                          * max(0.0, other[3] - other[1]))
            union = area + other_area - intersection
            if union > 0.0 and intersection / union >= iou_threshold:
                duplicate = True
                break
        if not duplicate:
            kept.append(index)
    return sorted(kept)
