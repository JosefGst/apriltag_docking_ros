class Docking:
    """
    checks if the tf is still detected.
    """
    
    def __init__(self, tf_not_detected_counter=3):
        self.count = 0 # count when no tf detected
        self.tf_not_detected_counter = tf_not_detected_counter # stop when no tf detected
        self.old_distance = 0
        

    def distance_no_change_count(self, distance):
        """
        counts how many times the distance hasen't changed

        :returns: count 
        """
        if distance == self.old_distance:
            self.count += 1
        else: 
            self.count = 0
        self.old_distance = distance
        return self.count
