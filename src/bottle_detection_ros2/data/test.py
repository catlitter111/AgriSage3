import cv2

def main():
    # 初始化摄像头
    cap = cv2.VideoCapture(1)  # 参数0通常代表默认摄像头
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 设置摄像头分辨率（可选）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        # 逐帧捕获
        ret, frame = cap.read()

        # 如果正确读取帧，ret为True
        if not ret:
            print("无法读取帧")
            break

        # 显示结果帧
        cv2.imshow('实时摄像头画面', frame)

        # 按'q'键退出循环
        if cv2.waitKey(1) == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()