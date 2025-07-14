import PerFrame
import cv2


def process_all(video_paths: list[str], output_path: str, fps: int = 30) -> None:
    """
    Process multiple video files in parallel and save the processed frames as a new video.
    Args:
        video_paths (list[str]): List of paths to video files.
        output_path (str): Path to save the output video.
        fps (int): Frames per second for the output video.
    """
    # Open all videos
    caps = [cv2.VideoCapture(path) for path in video_paths]

    num_frames = int(caps[0].get(cv2.CAP_PROP_FRAME_COUNT))  # Assuming all videos have same frame count

    # Read the first frame to get the size for VideoWriter
    initial_frames = []
    for cap in caps:
        ret, frame = cap.read()
        if not ret:
            raise RuntimeError("Failed to read the first frame from one of the videos.")
        initial_frames.append(frame)
    processed_frame = PerFrame.process_frame(*initial_frames)
    height, width = processed_frame.shape[:2]

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # type: ignore # Use 'XVID' for AVI
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    # Write the first processed frame
    out.write(processed_frame)

    for frame_idx in range(1, num_frames):
        frames = []
        for cap in caps:
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError(f"Failed to read frame {frame_idx} from one of the videos.")
            frames.append(frame)
        
        # Process the frames together
        processed_frame = PerFrame.process_frame(*frames)

        # Write the processed frame to the output video
        out.write(processed_frame)

    # Release resources
    for cap in caps:
        cap.release()
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Path to the input video files
    VIDEO_FILES = [
        "./input/unity-joints.mp4",
        "./input/unity-vid.mp4",
        "./input/kinect-joints.mp4",
        "./input/live-vid.mp4"
    ]

    # Path to save the output video
    OUTPUT_VIDEO = "./output/overlayed.mp4"

    # Process all videos
    process_all(VIDEO_FILES, OUTPUT_VIDEO)
