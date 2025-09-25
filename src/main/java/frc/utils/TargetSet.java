package frc.utils;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

public class TargetSet {
    List<PhotonTrackedTarget> currentTargets;

    public void setTargets(List<PhotonTrackedTarget> targets) {
        currentTargets = new LinkedList<PhotonTrackedTarget>();

        for (int i = 0; i < targets.size(); i++) {
            currentTargets.add(targets.get(i));
        }
    }

    public List<PhotonTrackedTarget> getTargets() {
        return currentTargets;
    }
}
