
class Docking:
    
    def __init__(self, count_max=3):
        self.count = 0 # count when no tf detected
        self.count_max = count_max # stop when no tf detected
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
