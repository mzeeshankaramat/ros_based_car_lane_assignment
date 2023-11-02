uint16 id                           # object ID
uint16 age                          # in how many scans was the object already seen
uint8 source_type                   # source type
uint8 classification                # classifiation of the object
string classification_str
float32 classification_certainty     # certainty about the right classification
float32 existence_confidence        # existence confidence of object
uint32 associated_sign              # id of sign with is associated with this object
float32 distance                    # distance of object in meter
box boundingBox                     # bounding box in image space

# classification (sign)
uint8 SIGN_CLASS_ADVISORYSPEEDLIMIT = 0
uint8 SIGN_CLASS_DIAMOND = 1
uint8 SIGN_CLASS_DONOTENTER = 2
uint8 SIGN_CLASS_ONEWAY = 3
uint8 SIGN_CLASS_REDRING = 4
uint8 SIGN_CLASS_EUBLUECIRCLE = 5
uint8 SIGN_CLASS_EUCIRCULARSHAPEPATTERN = 6
uint8 SIGN_CLASS_EUDIAMONDGENERAL = 7
uint8 SIGN_CLASS_EUREDRINGBLUECIRCLE = 8
uint8 SIGN_CLASS_SPEEDLIMIT10KMH = 9
uint8 SIGN_CLASS_SPEEDLIMIT15KMH = 10
uint8 SIGN_CLASS_SPEEDLIMIT20KMH = 11
uint8 SIGN_CLASS_SPEEDLIMIT30KMH = 12
uint8 SIGN_CLASS_SPEEDLIMIT40KMH = 13
uint8 SIGN_CLASS_SPEEDLIMIT50KMH = 14
uint8 SIGN_CLASS_SPEEDLIMIT60KMH = 15
uint8 SIGN_CLASS_SPEEDLIMIT70KMH = 16
uint8 SIGN_CLASS_SPEEDLIMIT80KMH = 17
uint8 SIGN_CLASS_SPEEDLIMIT90KMH = 18
uint8 SIGN_CLASS_SPEEDLIMIT100KMH = 19
uint8 SIGN_CLASS_SPEEDLIMIT110KMH = 20
uint8 SIGN_CLASS_SPEEDLIMIT120KMH = 21
uint8 SIGN_CLASS_SPEEDLIMIT130KMH = 22
uint8 SIGN_CLASS_EUVARMESGSPEEDLIMITGENERAL = 23
uint8 SIGN_CLASS_EUVARMESGWARNINGGENERAL = 24
uint8 SIGN_CLASS_EUVARIABLEMESSAGESIGN = 25
uint8 SIGN_CLASS_REDWARNINGTRIANGLE = 26
uint8 SIGN_CLASS_RESTRICTIONGENERAL = 27
uint8 SIGN_CLASS_SPEEDLIMITRESTRICTION = 28
uint8 SIGN_CLASS_STOP = 29
uint8 SIGN_CLASS_TURNSPROHIBITED = 30
uint8 SIGN_CLASS_YIELD = 31
uint8 SIGN_CLASS_UNSUPPORTED = 32