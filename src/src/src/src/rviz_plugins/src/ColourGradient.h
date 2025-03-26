#ifndef __COLOUR_GRADIENT_H__
#define __COLOUR_GRADIENT_H__

#include <Ogre.h>
#include <map>

/*!
@brief Colour gradient (linear interpolation).
@details This gradient class support "autofilling" of bounds values, ie, if no colour frame with index 0.0 is provided, it use a default black colour for this position of the gradient.
In the same way, if no colour frame is provided for 1.0 value, a default white colour is used for this position of the gradient.
Autobounds default colours (black/white) can be changed or using the appropriate constructor.
Disabling autobounds is also possible (no fading to autocolour), check the default constructor optionnal parameter.
@remark idea comes from SkyX, but totally rewritten in order to be more efficient.
*/
class ColourGradient
{
public:
protected:
private:
  const bool fillBounds;                                   //!< Should we fill bound of the gradient with default colours?
  typedef std::map<float, Ogre::ColourValue> ColourFrames; //!< ColourFrames data type
  ColourFrames colourFrames;                               //!< Colour frame map
  const Ogre::ColourValue minBoundDefaultColour;           //!< The default colour for min bound
  const Ogre::ColourValue maxBoundDefaultColour;           //!< The default colour for max bound
public:
  typedef ColourFrames::value_type ColourFrame;            //!< A single colour frame, first element of the pair is the position in the gradient, second element is the colour to insert in the gradient.

public:
  /*!
  Constructor. Gradient values range is [0.0, 1.0].
  @param fillBoundsWithDefault If true, and no colour frame with index 0.0 was provided, provide a default black colour for this position of the gradient. In the same way, if no colour frame is provided for 1.0 value, a default white colour is provided for this position of the gradient.
  */
  ColourGradient(bool fillBoundsWithDefault=true);

  /*!
  Constructor, bound filling is automatically activated when using this constructor. Gradient values range is [0.0, 1.0]
  @param overridedMinDefaultColour The default "black" colour used by automatic bound filling will be replaced by the provided colour
  @param overridedMaxDefaultColour
  */
  ColourGradient(const Ogre::ColourValue& overridedMinDefaultColour, const Ogre::ColourValue& overridedMaxDefaultColour);

  /*!
  Destructor 
  */
  ~ColourGradient();

  /*!
  @Add colour frame. Gradient values range is [0.0, 1.0]
  @param CFrame Colour frame
  */
  inline void AddColourFrame(const ColourFrame& colourFrame)
  {
    AddColourFrame(colourFrame.first, colourFrame.second);
  }

  /*!
  Add colour frame. Gradient values range is [0.0, 1.0]
  @param colour Coulour of this frame
  @param gradientPosition Position of this frame in the gradient (overwrite if value exists).
  */
  inline void AddColourFrame(const float& gradientPosition, const Ogre::ColourValue& colour)
  {
    assert((gradientPosition >= 0.0f)&&(gradientPosition<=1.0f));
    if(colourFrames.find(gradientPosition) == colourFrames.end())
      colourFrames.insert(ColourFrames::value_type(gradientPosition, colour));
    else
      colourFrames[gradientPosition] = colour;
  }

  /*!
  Clear colour gradient
  */
  inline void Clear()
  {
    colourFrames.clear();
  }

  /*!
  Get colour value
  @param p Gradient values range is [0.0, 1.0]
  @return Colour at the given gradient position
  */
  const Ogre::ColourValue GetColour(float gradientPosition) const;
protected:
private:
  /*!
  Interpolate the given colour frames.
  */
  Ogre::ColourValue Interpolate(const ColourFrame& minColourFrame, const ColourFrame& maxColourFrame, const float& mediumRangeValue) const;
};
#endif