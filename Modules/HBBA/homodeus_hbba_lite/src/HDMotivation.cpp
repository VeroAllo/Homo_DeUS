#include <homodeus_hbba_lite/HDMotivation.h>

using namespace std;

HDMotivation::HDMotivation(shared_ptr<DesireSet> desireSet, std::vector<Desire> desireList) : m_desireSet(move(desireSet)), m_desireList(move(desireList)) {}