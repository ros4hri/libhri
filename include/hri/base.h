//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef HRI_BASE_H
#define HRI_BASE_H

#include <memory>
#include <optional>
#include <string>
#include <ros/ros.h>

namespace hri
{
typedef std::string ID;

class FeatureTracker
{
public:
  FeatureTracker(ID id, const ros::NodeHandle& nh) : id_(id), node_(nh)
  {
  }

  virtual ~FeatureTracker()
  {
  }

  // forbids copies of our 'feature trackers', as we need to internally manage
  // if/when they disappear. Instead, access them via weak pointers (cf HRIListener API).

  // TODO: ask a C++ expert how to enable that while avoid compilation errors when
  // building/moving a FeatureTracker into a container (in HRIListener)
  FeatureTracker(const FeatureTracker&) = delete;



  ID getId() const
  {
    return id_;
  }

  virtual void init() = 0;

protected:
  ID id_;
  ros::NodeHandle node_;
};

}  // namespace hri

#endif
