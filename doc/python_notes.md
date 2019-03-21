


## Display

### pcl display
``` Python
    from matplotlib import pyplot as plt
    plt.subplot(121)
    plt.imshow(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    plt.subplot(122)
    plt.imshow(depth)
    plt.pause(0.001)
    plt.clf()
```