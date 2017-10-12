float32[2] KlGrad		#KeyLine's gradient vector [x,y]

float32[2] KlImgPos		#KeyLine's image position [x,y]

float64 invDepth		#Estimated Inverse Depth
float64 invDepthS		#Estimated Inverse Depth Uncertainty

float32[2] KlFocPos		#KL position in focal point centred coordinates (plane at focal length zf) [x,y]

int32 KlMatchID			#Id of the matching keyline

int32 ConsMatch			#number of consecutive matches

int16 KlPrevMatchID		#Id of previous consecutive KeyLine
int16 KlNextMatchID		#Id of next consecutive KeyLine