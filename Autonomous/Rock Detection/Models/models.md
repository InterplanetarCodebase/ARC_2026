- detection.pt:
	- Description: Very low accuracy. Poor performance and overall just made for testing. The reason is poor and narrow dataset. If dataset is made more vast it should perform better.

- segment_L_v1.pt:
	- Description: First large segmentation model trained with YOLOv26L-seg. Training was done for 100 epoch with normal learning rate. Performed really well by far.

- segment_L_v2.pt:
	- Description: Second large segmentation model trained with YOLOv26-seg. Training was done for 250 epoch with slower learning rate. Performance is not astronomically good than v1. Testing needed.

- segment_M.pt:
	- Description: First medium segmentation model trained with YOLOv26-seg. Training was done for 250 epoch with medium learning rate. Performance and load by far most balanced. Testing needed.
