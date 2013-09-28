#include "mirrorball.h"

using namespace std;

std::string Actuator_MirrorBall::mClassName = "Actuator_MirrorBall";
std::string Actuator_MirrorBall::mDocumentation = "N/A";
unsigned int Actuator_MirrorBall::mSourceNbr = 1;

/*************/
Actuator_MirrorBall::Actuator_MirrorBall()
{
    make();
}

/*************/
Actuator_MirrorBall::Actuator_MirrorBall(int pParam)
{
    make();
}

/*************/
void Actuator_MirrorBall::make()
{
    mOutputBuffer = cv::Mat::zeros(480, 640, CV_8UC3);

    mName = mClassName;
    // OSC path for this actuator
    mOscPath = "mirrorball";

    mFrameNumber = 0;

    mFOV = 45.f;
    mSphere = cv::Vec3f(0.f, 0.f, 0.f);
    mSphereDiameter = 1.f;
    mSphereReflectance = 1.f;
    mCameraDistance = 1.f;

    mFixedSphere = false;

    mTrackingLength = 30;
    mThreshold = 3.f;
}

/*************/
atom::Message Actuator_MirrorBall::detect(vector< Capture_Ptr > pCaptures)
{
    vector<cv::Mat> captures = captureToMat(pCaptures);
    if (captures.size() < mSourceNbr)
        return mLastMessage;
    cv::Mat capture = captures[0];
    
    mImage = capture;
    if (!mFixedSphere)
    {
        mSphere = detectSphere();
        mSphere = filterSphere(mSphere);
        if (mSphere[2] == 0.f)
            return mLastMessage;

        g_log(NULL, G_LOG_LEVEL_DEBUG, "%s - Sphere detected at position (%f, %f), radius %f", mClassName.c_str(), mSphere[0], mSphere[1], mSphere[2]);
    }

    int panoWidth = round(2 * M_PI * mSphere[2]);
    if (mEquiImage.cols != panoWidth)
        mEquiImage.create(panoWidth / 2, panoWidth, mImage.type());

    mSphereImage = capture(cv::Rect(mSphere[0]-mSphere[2], mSphere[1]-mSphere[2], mSphere[2]*2.f, mSphere[2]*2.f));
    if (mProjectionMap.total() == 0)
    {
        getDistanceFromCamera();
        createTransformationMap();
    }

    mSphereImage(cv::Rect(0, 0, 1, 1)).setTo(0);

    cv::Mat remappedImage;
    remap(mSphereImage, remappedImage, mProjectionMap, cv::Mat(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    mOutputBuffer = remappedImage;

    mFrameNumber++;
    mLastMessage = atom::createMessage("iii", 1, 1, mFrameNumber);

    return mLastMessage;
}

/*************/
void Actuator_MirrorBall::setParameter(atom::Message pMessage)
{
    std::string cmd;
    try
    {
        cmd = toString(pMessage[0]);
    }
    catch (atom::BadTypeTagError error)
    {
        return;
    }

    if (cmd == "sphereDiameter")
    {
        float size;
        if (readParam(pMessage, size))
            mSphereDiameter = max(0.f, size);
    }
    else if (cmd == "sphere")
    {
         cv::Vec3f sphere;
         if (!readParam(pMessage, sphere[0], 1))
            return;
         if (!readParam(pMessage, sphere[1], 2))
            return;
         if (!readParam(pMessage, sphere[2], 3))
            return;
         mSphere = sphere;
         mFixedSphere = true;
    }
    else if (cmd == "fov")
    {
        float fov;
        if (readParam(pMessage, fov))
            mFOV = max(0.1f, (float)(fov * M_PI / 180.f));
    }
    else if (cmd == "reflectance")
    {
        float reflectance;
        if (readParam(pMessage, reflectance))
            mSphereReflectance = max(0.f, min(1.f, reflectance));
    }
    else if (cmd == "trackingLength")
    {
        float length;
        if (readParam(pMessage, length))
            mTrackingLength = max(1, (int)length);
    }
    else if (cmd == "threshold")
    {
        float threshold;
        if (readParam(pMessage, threshold))
            mThreshold = threshold;
    }
    else
        setBaseParameter(pMessage);
}

/*************/
cv::Vec3f Actuator_MirrorBall::detectSphere()
{
    cv::Mat buffer;
    cv::cvtColor(mImage, buffer, CV_BGR2GRAY);
    cv::GaussianBlur(buffer, buffer, cv::Size(3, 3), 2, 2);
    cv::equalizeHist(buffer, buffer);

    vector<cv::Vec3f> circles;
    cv::HoughCircles(buffer, circles, CV_HOUGH_GRADIENT, 2, buffer.cols, 200, 150, buffer.rows / 8, buffer.rows);

    if (circles.size() == 0)
        return cv::Vec3f(0.f, 0.f, 0.f);

    if (circles[0][0] + circles[0][2] > mImage.cols || circles[0][0] - circles[0][2] < 0
        || circles[0][1] + circles[0][2] > mImage.rows || circles[0][1] - circles[0][2] < 0)
        return cv::Vec3f(0.f, 0.f, 0.f);

    return circles[0];
}

/*************/
cv::Vec3f Actuator_MirrorBall::filterSphere(cv::Vec3f newSphere)
{
    if (newSphere[2] != 0.f)
    {
        if (mSpherePositions.size() >= mTrackingLength)
            mSpherePositions.erase(mSpherePositions.begin());
        mSpherePositions.push_back(newSphere);
    }
    else if (mSpherePositions.size() == 0)
        return cv::Vec3f(0.f, 0.f, 0.f);

    cv::Vec3f sum(0.f, 0.f, 0.f);
    cv::Vec3f sum2(0.f, 0.f, 0.f);
    for (uint index = 0; index < (uint)mSpherePositions.size(); ++index)
    {
        sum += mSpherePositions[index];
        sum2 += mSpherePositions[index].mul(mSpherePositions[index]);
    }
    sum *= (1.f / (float)mSpherePositions.size());
    sum2 *= (1.f / (float)mSpherePositions.size());

    cv::Vec3f sigma;
    sigma[0] = sqrtf(sum2[0] - sum[0]*sum[0]);
    sigma[1] = sqrtf(sum2[1] - sum[1]*sum[1]);
    sigma[2] = sqrtf(sum2[2] - sum[1]*sum[1]);

    cv::Vec3f filtered(0.f, 0.f, 0.f);
    for (uint index = 0; index < (uint)mSpherePositions.size(); ++index)
    {
        if (mSpherePositions[index][0] - sum[0] > 2*sigma[0]
            || mSpherePositions[index][1] - sum[1] > 2*sigma[1]
            || mSpherePositions[index][2] - sum[2] > 2*sigma[2])
        {
            mSpherePositions.erase(mSpherePositions.begin() + index);
            index--;
        }
        else
            filtered += mSpherePositions[index];
    }

    filtered *= 1.f / (float)mSpherePositions.size();

    if (newSphere[2] != 0.f)
    {
        float distance = sqrtf(powf(newSphere[0] - filtered[0], 2.f) + powf(newSphere[1] - filtered[1], 2.f));
        float sigmaDist = sqrtf(sigma[0]*sigma[0] + sigma[1]*sigma[1]);
        if ((distance > mThreshold * sigmaDist) && !(mThreshold == 0.f))
        {
            mSpherePositions.clear();
            mSpherePositions.push_back(newSphere);
            return newSphere;
        }
    }

    return filtered;
}

/*************/
void Actuator_MirrorBall::getDistanceFromCamera()
{
    // It has been detected, so it's projected as a sphere
    // We consider here that the FOV is not to wide so that it's not
    // too much deformed.
    // Anyway, we compute a coeff to correct for not seeing the sphere
    // with an ortho projection
    // This is not a full correction but should suffice for normal lenses.

    // Compute the angle occupied by the sphere
    float  longitudeCoeff = tan(mFOV / 2.f) / ((float)mImage.cols / 2.f);

    float xleft = (mSphere[0] - mSphere[2] - (float)mImage.cols / 2.f) * longitudeCoeff;
    float xright = (mSphere[0] + mSphere[2] - (float)mImage.cols / 2.f) * longitudeCoeff;

    float alpha = atan(xright) - atan(xleft);
    alpha = abs(alpha);
    mCroppedFOV = alpha;

    // Now we compute the "real" radius from the viewed one
    float pixelRadius = mSphere[2] / cos(alpha / 2.f);

    // Calculation of the positioning error
    float lCorrection = mSphereDiameter / 2.f * sin(alpha / 2.f);

    // Compute the real FOV occupied by the equator of the sphere
    xleft = (mSphere[0] - pixelRadius - (float)mImage.cols / 2.f) * longitudeCoeff;
    xright = (mSphere[0] + pixelRadius - (float)mImage.cols / 2.f) * longitudeCoeff;
    alpha = atan(xright) - atan(xleft);
    alpha = abs(alpha);

    mCameraDistance = mSphereDiameter / (2.f * tan(alpha / 2.f)) + lCorrection;
    cout << mCameraDistance << endl;
}

/*************/
void Actuator_MirrorBall::createTransformationMap()
{
    cv::Size2f size = cv::Size2f(mSphereImage.size().width, mSphereImage.size().height);
    mProjectionMap.create(size, CV_32FC2);

    for (float x = 0.f; x < size.width; ++x)
        for (float y = 0.f; y < size.height; ++y)
        {
            // Calculating the view angle
            // Check if the pixel is on the mirrorball
            if (sqrt(pow(x - size.width/2.f, 2.f) + pow(y - size.height/2.f, 2.f)) > size.width / 2.f)
                mProjectionMap.at<cv::Vec2f>(y, x) = cv::Vec2f(M_PI, M_PI);
            else
            {
                float alpha, beta; // alpha along width, beta along height
                alpha = atanf(((x - size.width / 2.f) * mSphereDiameter / size.width) / mCameraDistance);
                beta = -atanf(((y - size.height / 2.f) * mSphereDiameter / size.height) / mCameraDistance);

                mProjectionMap.at<cv::Vec2f>(y, x) = directionFromViewAngle(cv::Vec2f(alpha, beta));
            }
        }

    projectionMapFromDirections();

    //vector<cv::Mat> channels;
    //cv::split(mProjectionMap, channels);
    //cv::imshow("c1", cv::abs(channels[0] / M_PI));
    //cv::imshow("c2", cv::abs(channels[1] / M_PI));
}

/*************/
cv::Vec2f Actuator_MirrorBall::directionFromViewAngle(cv::Vec2f angle)
{
    // All equations are the result of a simplification of the source equations, done with Matlab
    // TODO: find this Matlab file and put it in the documentation

    float a, b; // for lisibility, we will use these vars for angles...
    float c; // ... and this one for distance to the camera...
    float r; // ... and this one for the sphere radius
    a = angle[0];
    b = angle[1];
    c = -mCameraDistance;
    r = mSphereDiameter / 2.f;

    // Precomputing some useful values
    float cosA = cosf(a);
    float cosB = cosf(b);
    float cos2a = cosf(2*a);
    float cos2b = cosf(2*b);

    // We first verify that the pixel is on the sphere
    // (although this test should have been done before)
    float delta = cosA*cosA*cosB*cosB * (-2.f*c*c + 3.f*r*r + r*r*cos2b + cos2a*(r*r + (2.f*c*c - r*r) * cos2b));
    if (delta < 0.f)
        return cv::Vec2f(0.f, 0.f);

    // We can safely calculate the t parameter
    float num, denom;
    num = 4.f*c + 4.f*c*cos2a + 2.f*c*cosf(2.f*a - 2.f*b) + 4.f*c*cos2b + 2.f*c*cosf(2.f*(a+b));
    denom = 2.f * (-6.f - 2.f*cos2a + cosf(2.f*a - 2.f*b) - 2.f*cos2b + cosf(2.f*(a+b)));
    float t;
    if (denom > 0.f)
        t = (num - 8.f * sqrtf(delta)) / denom;
    else
        t = (num + 8.f * sqrtf(delta)) / denom;

    // We calculate the point of impact
    float x, y, z;
    x = c + t;
    y = t * tanf(a);
    z = t * tanf(b);

    // Now we create a base to project the ray from the camera onto
    cv::Vec3f u, v, w;
    u = cv::Vec3f(x, y, z) * (1.f / r);
    v = cv::Vec3f(u[1], -u[0], 0.f);
    w = u.cross(v);

    // Projection onto this new base
    cv::Vec3f newCoords;
    cv::Vec3f inputDir = cv::Vec3f(x - c, y, z);
    newCoords[0] = inputDir.dot(u);
    newCoords[1] = inputDir.dot(v);
    newCoords[2] = inputDir.dot(w);

    // The reflected ray has the same direction, except on the axis
    // which is normal to the sphere for which it has the opposite value
    newCoords[0] = -newCoords[0];

    // We reproject the direction into the old base
    cv::Vec3f outputDir = newCoords[0]*u + newCoords[1]*v + newCoords[2]*w;

    // Calculate Euler angles from this new direction
    outputDir = outputDir * (1 / sqrtf(outputDir.dot(outputDir)));

    float yaw, pitch;
    if (outputDir[0] == 0.f)
        if (outputDir[1] > 0.f)
            yaw = M_PI_2;
        else
            yaw = -M_PI_2;
    else if (outputDir[0] > 0.f)
        if (outputDir[1] < 0.f)
            yaw = asinf(-outputDir[1] / sqrtf(outputDir[0] * outputDir[0] + outputDir[1] * outputDir[1]));
        else
            yaw = 2 * M_PI + asinf(-outputDir[1] / sqrtf(outputDir[0] * outputDir[0] + outputDir[1] * outputDir[1]));
    else
        yaw = M_PI - asinf(-outputDir[1] / sqrtf(outputDir[0] * outputDir[0] + outputDir[1] * outputDir[1]));

    pitch = asinf(outputDir[2]);

    return cv::Vec2f(yaw - M_PI, pitch);
}

/*************/
void Actuator_MirrorBall::projectionMapFromDirections()
{
    cv::Size2f size = cv::Size2f(mSphereImage.size().width, mSphereImage.size().height);

    float wCoeff = size.width / (2 * M_PI);
    float hCoeff = size.height / (2 * M_PI);

    for (int x = 0; x < size.width; ++x)
        for (int y = 0; y < size.height; ++y)
        {
            mProjectionMap.at<cv::Vec2f>(y, x)[0] = (mProjectionMap.at<cv::Vec2f>(y, x)[0] + M_PI) * hCoeff;
            mProjectionMap.at<cv::Vec2f>(y, x)[1] = (mProjectionMap.at<cv::Vec2f>(y, x)[1] + M_PI) * wCoeff;
        }

    // We have a forward map: cv::remap needs a backward one
    // TODO: find a more elegant way to do that, especially regarding the smoothing
    cv::Mat backMap = cv::Mat::zeros(mProjectionMap.size(), mProjectionMap.type());
    cv::Mat mask = cv::Mat::zeros(mProjectionMap.size(), CV_8U);

    for (int x = 0; x < size.width; ++x)
        for (int y = 0; y < size.height; ++y)
        {
            if (mProjectionMap.at<cv::Vec2f>(y, x)[0] < mSphereImage.rows-1 && mProjectionMap.at<cv::Vec2f>(y, x)[1] < mSphereImage.cols-1)
            {
                int u, v;
                u = round(mProjectionMap.at<cv::Vec2f>(y, x)[0]);
                v = round(mProjectionMap.at<cv::Vec2f>(y, x)[1]);

                backMap.at<cv::Vec2f>(v, u)[0] = mProjectionMap.cols - x;
                backMap.at<cv::Vec2f>(v, u)[1] = mProjectionMap.rows - y;

                mask.at<uchar>(v, u) = 255;
            }
        }

    mask = 255 - mask;
    cv::Mat buffer = cv::Mat::zeros(mProjectionMap.size(), mProjectionMap.type());
    cv::dilate(backMap, buffer, cv::Mat(), cv::Point(-1, -1), 4);
    mProjectionMap = backMap;
    cv::add(mProjectionMap, buffer, mProjectionMap, mask, mProjectionMap.type());

    cv::resize(buffer, buffer, cv::Size(mEquiImage.cols, mEquiImage.cols));
    mProjectionMap = buffer(cv::Rect(0, (buffer.rows - mEquiImage.rows) / 2, buffer.cols, mEquiImage.rows));
}
