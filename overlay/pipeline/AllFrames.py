import PerFrame
import cv2
import imageio.v2 as imageio


def process_all(video_paths: list[str], output_path: str, fps: int = 30) -> None:
    """
    Process multiple video files in parallel and save the processed frames as a new video using imageio.
    Args:
        video_paths (list[str]): List of paths to video files.
        output_path (str): Path to save the output video.
        fps (int): Frames per second for the output video.
    """

    # Open all videos
    caps = [cv2.VideoCapture(path) for path in video_paths]
    num_frames = int(caps[0].get(cv2.CAP_PROP_FRAME_COUNT))  # Assuming all videos have same frame count

    processed_frames = []
    for frame_idx in range(num_frames):
        frames = []
        for cap in caps:
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError(f"Failed to read frame {frame_idx} from one of the videos.")
            frames.append(frame)
        
        # Process the frames together
        processed_frame = PerFrame.process_frame(*frames)
        processed_frames.append(processed_frame)
        
    # Release resources
    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()

    # Write video using imageio, which expects frames in RGB order
    imageio.mimsave(output_path, processed_frames, fps=fps)


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
