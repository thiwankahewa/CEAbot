# =============================================================
    # Data capture helpers
    # =============================================================

    def capture_current_data(self, tag: str = ""):
        """
        First-step test capture:
        save RGB image and optionally depth image.

        In step 2, add:
        - plant segmentation
        - canopy width extraction
        - plant height extraction from depth
        """
        timestamp = int(time.time() * 1000)

        if self.get_parameter("save_images").value and self.latest_rgb_cv is not None:
            rgb_path = os.path.join(self.image_save_dir, f"{timestamp}_{tag}_rgb.png")
            cv2.imwrite(rgb_path, self.latest_rgb_cv)
            self.get_logger().info(f"Saved RGB image: {rgb_path}")

        if self.get_parameter("save_depth").value and self.latest_depth_cv is not None:
            depth_path = os.path.join(self.image_save_dir, f"{timestamp}_{tag}_depth.npy")
            np.save(depth_path, self.latest_depth_cv)
            self.get_logger().info(f"Saved depth image array: {depth_path}")

        # ---------------------------------------------------------
        # STEP 2 PLACEHOLDER: phenotyping starts here
        # ---------------------------------------------------------
        # Example future pipeline:
        #
        # if self.latest_rgb_cv is not None:
        #     plant_mask = self.segment_plant(self.latest_rgb_cv)
        #     canopy_width_across_px, canopy_width_along_px = self.measure_canopy_widths(plant_mask)
        #
        # if self.latest_depth_cv is not None:
        #     plant_height_m = self.estimate_plant_height(self.latest_rgb_cv, self.latest_depth_cv)
        #
        # Save results to CSV / publish ROS topics here
        # ---------------------------------------------------------

    # =============================================================
    # Future phenotyping stubs
    # =============================================================

    def segment_plant(self, rgb_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        FUTURE STEP 2:
        Replace with your tomato canopy segmentation.
        Could start with HSV green thresholding.
        """
        hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)

        # Example only - tune later
        lower = np.array([35, 40, 40], dtype=np.uint8)
        upper = np.array([90, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def measure_canopy_widths(self, plant_mask: np.ndarray) -> Tuple[float, float]:
        """
        FUTURE STEP 2:
        Placeholder for canopy width extraction.
        Return widths in pixels for now.
        Later convert to metric units using calibration / depth.
        """
        ys, xs = np.where(plant_mask > 0)
        if len(xs) == 0:
            return 0.0, 0.0

        width_across_px = float(xs.max() - xs.min())
        width_along_px = float(ys.max() - ys.min())
        return width_across_px, width_along_px

    def estimate_plant_height(self, rgb_bgr: np.ndarray, depth_img: np.ndarray) -> float:
        """
        FUTURE STEP 2:
        Placeholder for height estimation.
        Use plant mask + depth + known reference plane.
        """
        # Example dummy return
        return 0.0
