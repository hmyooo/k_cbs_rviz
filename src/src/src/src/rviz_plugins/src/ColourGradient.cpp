#include "ColourGradient.h"

ColourGradient::ColourGradient(bool fillBoundsWithDefault) : fillBounds(fillBoundsWithDefault),
                                                             minBoundDefaultColour(Ogre::ColourValue::Black),
                                                             maxBoundDefaultColour(Ogre::ColourValue::White)
{
}

ColourGradient::ColourGradient(const Ogre::ColourValue& overridedMinDefaultColour, const Ogre::ColourValue& overridedMaxDefaultColour) : fillBounds(true),
                                                                                                                                         minBoundDefaultColour(overridedMinDefaultColour),
                                                                                                                                         maxBoundDefaultColour(overridedMaxDefaultColour)
{
}

ColourGradient::~ColourGradient()
{
}

const Ogre::ColourValue ColourGradient::GetColour(float gradientPosition) const
{
  // Check parameter
  if(gradientPosition < 0.0f)
    gradientPosition = 0.0f;
  if(gradientPosition > 1.0f)
    gradientPosition = 1.0f;

  // No colours in the gradient!
  if (colourFrames.empty())
  {
    if(!fillBounds)
      return Ogre::ColourValue::Black;
    else
      return Interpolate(ColourFrame(0.0f, minBoundDefaultColour), ColourFrame(1.0f, maxBoundDefaultColour), gradientPosition);
  }

  // If the requested colour is exactly on a colour frame position
  ColourFrames::const_iterator iSearchedColourFrame = colourFrames.find(gradientPosition);
  if(iSearchedColourFrame != colourFrames.end())
    return iSearchedColourFrame->second;

  // Only one colour
  if (colourFrames.size() == 1)
  {
    if(!fillBounds)
    {
      // No bound filling, return the unique colour.
      return colourFrames.begin()->second;
    }
    else
    {
      if(gradientPosition < colourFrames.begin()->first)
      {
        // Interpolate between default colour frame (0.0, Ogre::ColourValue::Black) and unique colour that is in the gradient.
        return Interpolate(ColourFrame(0.0f, minBoundDefaultColour), *colourFrames.begin(), gradientPosition);
      }
      else
      {
        // Interpolate between the unique colour that is in the gradient and default colour frame (1.0, Ogre::ColourValue::White).
        return Interpolate(*colourFrames.begin(), ColourFrame(1.0f, maxBoundDefaultColour), gradientPosition);
      }
    }
  }

  // Min colour value
  ColourFrames::const_iterator iMinBound = colourFrames.lower_bound(gradientPosition);
  if(iMinBound != colourFrames.begin())
    iMinBound--;

  // Max value
  ColourFrames::const_iterator iMaxBound = colourFrames.upper_bound(gradientPosition);

  // Handle the case where the requested value is below the smaller value in the gradient
  if(gradientPosition < iMinBound->first)
  {
    if(!fillBounds)
    {
      // No bound filling, return the colour which is just above the searched value.
      return iMinBound->second;
    }
    else
    {
      // Interpolate between default colour frame (0.0, Ogre::ColourValue::Black) and the colour just above the searched value.
      return Interpolate(ColourFrame(0.0f, minBoundDefaultColour), *iMinBound, gradientPosition);
    }
  }

  // Handle the case where the requested value is above the greater value in the gradient
  if(iMaxBound == colourFrames.end())
  {
    iMaxBound--;
    if(!fillBounds)
    {
      // No bound filling, return the colour which is just below the searched value.
      return iMaxBound->second;
    }
    else
    {
      // Interpolate between the colour just below the searched value and the default colour frame (1.0, Ogre::ColourValue::White).
      return Interpolate(*iMaxBound, ColourFrame(1.0f, maxBoundDefaultColour), gradientPosition);
    }
  }

  // Normal case
  return Interpolate(*iMinBound, *iMaxBound, gradientPosition);
}

Ogre::ColourValue ColourGradient::Interpolate(const ColourFrame& minColourFrame, const ColourFrame& maxColourFrame, const float& mediumRangeValue) const
{
  float range = maxColourFrame.first - minColourFrame.first;
  float rangePoint = (mediumRangeValue - minColourFrame.first) / range;
  return ((minColourFrame.second * (1.0f - rangePoint)) + (maxColourFrame.second * rangePoint));
}
