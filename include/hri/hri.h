
#ifndef HRI_HRI_H
#define HRI_HRI_H

#include <optional>
#include <string>

namespace hri {

typedef std::optional<std::string> id;

class Person {
   public:
    id getFaceID() const { return face_id; }
    id getBodyID() const { return body_id; }
    id getVoiceID() const { return voice_id; }

   protected:
    id face_id;
    id body_id;
    id voice_id;
};

}  // namespace hri

#endif  // HRI_HRI_H
