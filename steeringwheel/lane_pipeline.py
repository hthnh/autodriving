import cv2
import numpy as np

# --- Configuration Constants (MUST BE TUNED) ---

# [REVISED] HSV Color Thresholds for White and Yellow lines
# This is a new strategy based on the sample image.

# 1. White mask (Low Saturation, Very High Value for bright light)
LOWER_WHITE = np.array([0, 0, 200])
UPPER_WHITE = np.array([255, 40, 255])

# 2. Yellow mask (Hue between ~20-35, high Saturation and Value)
LOWER_YELLOW = np.array([20, 100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

# [REVISED] Perspective Transform Points
# Adjusted based on the sample image (vanishing point higher, avoid car hood)
SRC_POINTS_PERCENT = {
    "top_left": (0.45, 0.6),      # Moved y up from 0.65
    "top_right": (0.55, 0.6),     # Moved y up from 0.65
    "bottom_left": (0.1, 0.95),   # Moved y up from 1.0 (avoid hood)
    "bottom_right": (0.9, 0.95)   # Moved y up from 1.0 (avoid hood)
}

# 3. Sliding Window Parameters (Unchanged)
N_WINDOWS = 9
WINDOW_MARGIN = 100
MIN_PIXELS = 50




# --- [NEW] Helper Function: Global Brightness Normalization ---

def normalize_brightness(frame, ref_mean, ref_std):
    """
    Normalizes the brightness of a frame to match a reference mean and std_dev.
    This is based on the Reinhard color transfer method, applied only to
    the V (Value) channel.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    curr_mean = np.mean(v)
    curr_std = np.std(v)
    
    # Avoid division by zero if std_dev is 0 (e.g., solid black frame)
    if curr_std == 0:
        curr_std = 1 
            
    # Apply normalization: v_norm = (v - mean) * (ref_std / std) + ref_mean
    v_normalized = (v - curr_mean) * (ref_std / curr_std) + ref_mean
    
    # Clip values to the valid 0-255 range
    v_normalized = np.clip(v_normalized, 0, 255).astype(np.uint8)
    
    # Merge channels and convert back to BGR
    hsv_normalized = cv2.merge([h, s, v_normalized])
    bgr_normalized = cv2.cvtColor(hsv_normalized, cv2.COLOR_HSV2BGR)
    
    return bgr_normalized




# --- Helper Function: Step 2 (Adaptive Filtering) ---
# [MODIFIED] Now uses CLAHE to normalize brightness before color thresholding.

def adaptive_color_filter(frame):
    """
    Applies HSV filtering to find white and yellow lane candidates.
    [NEW] Uses CLAHE (Contrast Limited Adaptive Histogram Equalization)
    to handle severe shadows and bright spots.
    """
    height, width = frame.shape[:2]

    # --- [NEW] CLAHE Brightness Normalization ---
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Split the channels
    h, s, v = cv2.split(hsv)
    
    # Create a CLAHE object
    # clipLimit determines the max contrast amplification (prevents noise)
    # tileGridSize is the 8x8 grid we apply equalization to.
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16))
    
    # Apply CLAHE to the V (Value) channel
    v_equalized = clahe.apply(v)
    
    # Merge the equalized V channel back with H and S
    hsv_equalized = cv2.merge([h, s, v_equalized])
    # --- End of CLAHE block ---

    # Create white mask (using the *equalized* HSV image)
    white_mask = cv2.inRange(hsv_equalized, LOWER_WHITE, UPPER_WHITE)
    
    # Create yellow mask (using the *equalized* HSV image)
    yellow_mask = cv2.inRange(hsv_equalized, LOWER_YELLOW, UPPER_YELLOW)
    
    # Combine masks
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    
    # Mask the top 50% of the image (sky)
    top_mask_height = height // 2
    cv2.rectangle(combined_mask, (0, 0), (width, top_mask_height), 0, -1)
    
    return combined_mask



# --- Helper Function: Step 3 (Perspective Transform) ---
# (Unchanged from previous version, but will use new SRC_POINTS_PERCENT)

def perspective_transform(image):
    """
    Applies a perspective transform (Bird's-Eye View) to the image.
    Returns the warped image, the transform matrix (M),
    and the inverse transform matrix (Minv).
    """
    height, width = image.shape[:2]

    # Uses the global SRC_POINTS_PERCENT defined above
    src = np.float32([
        (width * SRC_POINTS_PERCENT["top_left"][0], height * SRC_POINTS_PERCENT["top_left"][1]),
        (width * SRC_POINTS_PERCENT["top_right"][0], height * SRC_POINTS_PERCENT["top_right"][1]),
        (width * SRC_POINTS_PERCENT["bottom_left"][0], height * SRC_POINTS_PERCENT["bottom_left"][1]),
        (width * SRC_POINTS_PERCENT["bottom_right"][0], height * SRC_POINTS_PERCENT["bottom_right"][1])
    ])

    dst_bottom_left_x = width * 0.1
    dst_bottom_right_x = width * 0.9
    
    dst = np.float32([
        (dst_bottom_left_x, 0),
        (dst_bottom_right_x, 0),
        (dst_bottom_left_x, height - 1),
        (dst_bottom_right_x, height - 1)
    ])

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    
    warped_image = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
    
    return warped_image, M, Minv

# --- Helper Function: Step 4 & 5 (Find Lanes & Fit Poly) ---
# (Unchanged from previous version)

def find_lanes_sliding_window(warped_mask):
    """
    Finds lane pixels using histogram and sliding windows, then fits a
    second-order polynomial (x = Ay^2 + By + C) to each.
    
    Returns: (left_fit, right_fit, out_img)
    'out_img' is a visualization of the windows and fitted lines.
    """
    height, width = warped_mask.shape

    out_img = np.dstack((warped_mask, warped_mask, warped_mask))

    histogram = np.sum(warped_mask[height//2:, :], axis=0)
    midpoint = width // 2
    
    # Handle cases where one side might be missing (e.g., shadows)
    # We set a minimum peak search distance from the edges
    peak_margin = 50
    left_base_range = histogram[peak_margin:midpoint]
    
    if len(left_base_range) > 0:
        left_base = np.argmax(left_base_range) + peak_margin
    else:
        left_base = peak_margin # Default
        
    right_base_range = histogram[midpoint:-peak_margin]
    if len(right_base_range) > 0:
        right_base = np.argmax(right_base_range) + midpoint
    else:
        right_base = width - peak_margin # Default

    window_height = height // N_WINDOWS
    nonzero_y, nonzero_x = warped_mask.nonzero()

    left_current = left_base
    right_current = right_base

    left_lane_inds = []
    right_lane_inds = []

    for window in range(N_WINDOWS):
        win_y_low = height - (window + 1) * window_height
        win_y_high = height - window * window_height
        
        win_x_left_low = left_current - WINDOW_MARGIN
        win_x_left_high = left_current + WINDOW_MARGIN
        win_x_right_low = right_current - WINDOW_MARGIN
        win_x_right_high = right_current + WINDOW_MARGIN

        cv2.rectangle(out_img, (win_x_left_low, win_y_low), (win_x_left_high, win_y_high), (0, 255, 0), 2)
        cv2.rectangle(out_img, (win_x_right_low, win_y_low), (win_x_right_high, win_y_high), (0, 255, 0), 2)

        good_left_inds = (
            (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
            (nonzero_x >= win_x_left_low) & (nonzero_x < win_x_left_high)
        ).nonzero()[0]
        
        good_right_inds = (
            (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
            (nonzero_x >= win_x_right_low) & (nonzero_x < win_x_right_high)
        ).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > MIN_PIXELS:
            left_current = int(np.mean(nonzero_x[good_left_inds]))
        if len(good_right_inds) > MIN_PIXELS:
            right_current = int(np.mean(nonzero_x[good_right_inds]))

    try:
        left_lane_inds = np.concatenate(left_lane_inds)
    except ValueError:
        pass # Keep empty
    try:
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        pass # Keep empty

    left_x = nonzero_x[left_lane_inds]
    left_y = nonzero_y[left_lane_inds]
    right_x = nonzero_x[right_lane_inds]
    right_y = nonzero_y[right_lane_inds]

    if len(left_y) == 0 or len(left_x) == 0:
        left_fit = np.array([0, 0, left_base])
    else:
        left_fit = np.polyfit(left_y, left_x, 2)

    if len(right_y) == 0 or len(right_x) == 0:
        right_fit = np.array([0, 0, right_base])
    else:
        right_fit = np.polyfit(right_y, right_x, 2)

    out_img[nonzero_y[left_lane_inds], nonzero_x[left_lane_inds]] = [255, 0, 0]
    out_img[nonzero_y[right_lane_inds], nonzero_x[right_lane_inds]] = [0, 0, 255]

    plot_y = np.linspace(0, height - 1, height)
    try:
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]
        
        pts_left = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
        pts_right = np.array([np.transpose(np.vstack([right_fit_x, plot_y]))])
        
        cv2.polylines(out_img, np.int_([pts_left]), isClosed=False, color=(255, 255, 0), thickness=2)
        cv2.polylines(out_img, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=2)
    
    except np.linalg.LinAlgError:
        pass

    return left_fit, right_fit, out_img

# --- Main Pipeline Function: Step 6 (Calculate Error) ---
# [MODIFIED] Now accepts reference stats and performs 2-stage processing.

def process_frame(frame, ref_mean, ref_std):
    """
    Executes the full lane detection pipeline on a single frame.
    [NEW] Performs global normalization (Step 1) before
    local adaptation (Step 2).
    
    Returns:
    - error (float)
    - combined_mask (image)
    - window_viz_img (image)
    - final_result_img (image)
    """
    height, width = frame.shape[:2]
    blank_image = np.zeros_like(frame)

    # --- [NEW] Step 1: Global Brightness Normalization ---
    # This tames extreme global brightness (like full sun)
    try:
        normalized_frame = normalize_brightness(frame, ref_mean, ref_std)
    except cv2.error:
        print("Error during brightness normalization.")
        normalized_frame = frame # Fallback to original

    # --- Step 2: Adaptive Filtering (CLAHE, etc.) ---
    # This step now runs on the *normalized* frame
    try:
        combined_mask = adaptive_color_filter(normalized_frame)
    except cv2.error:
        print("Error in color filter.")
        return 0.0, np.zeros_like(frame, dtype=np.uint8), blank_image, frame

    # Step 3: Perspective Transform
    try:
        warped_mask, M, Minv = perspective_transform(combined_mask)
    except (np.linalg.LinAlgError, cv2.error):
        print("Warning: Perspective transform failed.")
        return 0.0, combined_mask, blank_image, frame

    # Step 4 & 5: Find Lanes & Fit Polynomial
    try:
        left_fit, right_fit, window_viz_img = find_lanes_sliding_window(warped_mask)
    except (np.linalg.LinAlgError, TypeError, ValueError):
        print("Warning: Polynomial fitting failed.")
        return 0.0, combined_mask, blank_image, frame

    # Step 6: Calculate Error (Unchanged)
    y_eval = height - 1
    left_x = left_fit[0] * (y_eval**2) + left_fit[1] * y_eval + left_fit[2]
    right_x = right_fit[0] * (y_eval**2) + right_fit[1] * y_eval + right_fit[2]
    lane_center = (left_x + right_x) / 2
    image_center = width / 2
    error = lane_center - image_center
    
    # --- Create Final Lane Overlay Image ---
    try:
        warp_zero = np.zeros_like(warped_mask).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        plot_y = np.linspace(0, height - 1, height)
        left_fit_x = left_fit[0] * plot_y**2 + left_fit[1] * plot_y + left_fit[2]
        right_fit_x = right_fit[0] * plot_y**2 + right_fit[1] * plot_y + right_fit[2]

        pts_left = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))])
        pts = np.hstack((pts_left, pts_right))

        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        new_warp = cv2.warpPerspective(color_warp, Minv, (width, height))
        
        # [MODIFIED] Ensure we add the overlay to the *ORIGINAL* frame,
        # not the normalized one, for correct visualization.
        final_result_img = cv2.addWeighted(frame, 1, new_warp, 0.3, 0)
        
    except (np.linalg.LinAlgError, cv2.error, ValueError):
        print("Warning: Failed to draw final overlay.")
        final_result_img = frame # Return original frame on draw failure

    return error, combined_mask, window_viz_img, final_result_img











# --- Video Processing Function ---
# [MODIFIED] Now calculates reference brightness stats from the first frame.

def process_video(video_input_path):
    """
    Opens a video file, calculates reference brightness,
    processes each frame, and displays a 2x2 debug dashboard.
    """
    cap = cv2.VideoCapture(video_input_path)
    
    if not cap.isOpened():
        print(f"Error: Could not open video file at {video_input_path}")
        return

    # --- [NEW] Get reference stats from the first frame ---
    ret, first_frame = cap.read()
    if not ret:
        print("Error: Could not read the first frame.")
        cap.release()
        return
        
    try:
        hsv_ref = cv2.cvtColor(first_frame, cv2.COLOR_BGR2HSV)
        _, _, v_ref = cv2.split(hsv_ref)
        REF_MEAN = np.mean(v_ref)
        REF_STD = np.std(v_ref)
        print(f"Reference brightness set: Mean={REF_MEAN:.2f}, StdDev={REF_STD:.2f}")
    except cv2.error as e:
        print(f"Error processing first frame: {e}")
        cap.release()
        return

    # [NEW] Reset video capture to start from the beginning
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
    # --- End of reference setup ---

    frame_count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        
        if not ret:
            print("Processing finished.")
            break
        
        frame_count += 1
        
        # [MODIFIED] Run the full pipeline, passing in the reference stats
        error_pixels, mask, viz, final = process_frame(frame, REF_MEAN, REF_STD)
        
        print(f"Frame {frame_count}: Error = {error_pixels:.2f} pixels")

        # --- Build the 2x2 Debug Dashboard ---
        # (This section is unchanged as it now correctly uses
        # 'frame' for original and 'final' for original+overlay)
        try:
            h, w = frame.shape[:2]
            h_small, w_small = h // 2, w // 2
            
            frame_small = cv2.resize(frame, (w_small, h_small))
            final_small = cv2.resize(final, (w_small, h_small))
            
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_small = cv2.resize(mask_bgr, (w_small, h_small))
            
            if viz.shape[0] != h or viz.shape[1] != w:
                viz = np.zeros_like(frame)
                
            viz_small = cv2.resize(viz, (w_small, h_small))
            
            top_row = np.hstack((frame_small, final_small))
            bottom_row = np.hstack((mask_small, viz_small))
            
            dashboard = np.vstack((top_row, bottom_row))

            cv2.imshow('Debug Dashboard (Press Q to quit)', dashboard)
        
        except cv2.error as e:
            print(f"Error during dashboard creation: {e}")
            cv2.imshow('Debug Dashboard (Press Q to quit)', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
    
    
    
    
    
    
    
    
    
    # --- Entry Point ---

if __name__ == "__main__":
    INPUT_VIDEO = "input_video.mp4"
    
    print(f"Starting lane finding pipeline for: {INPUT_VIDEO}")
    print("A 'Debug Dashboard' window will open. Press 'q' to stop processing.")
    process_video(INPUT_VIDEO)