# launch-vehicle-tracker
Designing a robust and real time vehicle tracker that allows a raspberry pi camera to track the rocket through changing luminosity, smoke occlusion, etc.

#### Tentative Algorithm Design

*Correlation Filters* are one of the most commonly used and robust methods in object tracking to get past the problems of partial occlusion, and luminosity, and as such, seems like the best way forward when it comes to a challenging target like a rocket. [Further reading](https://dl.acm.org/doi/book/10.5555/2520035#:~:text=Correlation%20Filters%20are%20a%20class,localization%20of%20targets%20in%20scenes.&text=First%2C%20traditional%20correlation%20filter%20designs,scalar%20feature%20representations%20of%20objects,)

On a high level, the tentative algorithm works as follows:

1. First, we find our *area of interest*, that is the area we want to track within the image.
2. We craft a known response from that area of interest. We can think of this as a function F(x) where when we pass in an image x to the function, we get a known response Y, representing the center of mass of where we are tracking.
3. The function F(x) is really the output of **correlating** ALL circular shifts of the image X with a unknown filter H that gets us the response Y, so we can think of it as a function F(x, H).
4. Our goal is to find this filter H such that even through occlusion and lighting changes, F(X) (this involves H) still gives us the response Y we want.
5. Initially, we don't know H. What we do is take the first couple of frames and manually generate a known response. What this allows us to do is take a random filter and minimize the squared error between F(

This essentially takes all the frequencies of the image and writes them as intensity values to a grey scale image. We can sort of think of this as analyzing the varying intensity of the original pixel values, taking out the base frequencies 






