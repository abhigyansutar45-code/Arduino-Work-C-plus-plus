/*
 * ================================================================
 *  AUTONOMOUS SELF-BALANCING ROBOT — ASBR
 * ================================================================
 *
 *  PART 2
 *
 *  OCCUPANCY GRID
 *  RAY CASTING
 *  SENSOR PROJECTION
 *  MAP UPDATE ENGINE
 *  MAP STATISTICS
 *
 *  CONTINUES FROM PART 1
 *
 * ================================================================
 */


/*
 * ================================================================
 * SECTION 40 — MAP CELL STATES
 * ================================================================
 */

enum class MapCellState : int8_t
{
    UNKNOWN = -1,
    FREE = 0,
    OCCUPIED = 1
};


/*
 * ================================================================
 * SECTION 41 — MAP CELL
 * ================================================================
 */

struct MapCell
{
    int8_t occupancy;

    uint16_t observations;

    uint16_t freeObservations;

    uint16_t occupiedObservations;

    MapCell()
        : occupancy(
              static_cast<int8_t>(
                  MapCellState::UNKNOWN
              )
          ),
          observations(0),
          freeObservations(0),
          occupiedObservations(0)
    {
    }

    void reset()
    {
        occupancy =
            static_cast<int8_t>(
                MapCellState::UNKNOWN
            );

        observations = 0;

        freeObservations = 0;

        occupiedObservations = 0;
    }

    bool isUnknown() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::UNKNOWN
            );
    }

    bool isFree() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::FREE
            );
    }

    bool isOccupied() const
    {
        return occupancy ==
            static_cast<int8_t>(
                MapCellState::OCCUPIED
            );
    }
};


/*
 * ================================================================
 * SECTION 42 — GRID COORDINATE
 * ================================================================
 */

struct GridCoordinate
{
    int x;

    int y;

    GridCoordinate()
        : x(0),
          y(0)
    {
    }

    GridCoordinate(
        int gridX,
        int gridY
    )
        : x(gridX),
          y(gridY)
    {
    }

    bool operator==(
        const GridCoordinate& other
    ) const
    {
        return
            x == other.x &&
            y == other.y;
    }

    bool operator!=(
        const GridCoordinate& other
    ) const
    {
        return !(*this == other);
    }
};


/*
 * ================================================================
 * SECTION 43 — SENSOR RAY
 * ================================================================
 */

struct SensorRay
{
    Vector2 origin;

    Vector2 endpoint;

    float angle;

    float distance;

    bool valid;

    SensorRay()
        : origin(),
          endpoint(),
          angle(0.0f),
          distance(0.0f),
          valid(false)
    {
    }
};


/*
 * ================================================================
 * SECTION 44 — MAP BOUNDARIES
 * ================================================================
 */

struct MapBounds
{
    float minimumX;

    float maximumX;

    float minimumY;

    float maximumY;

    MapBounds()
        : minimumX(0.0f),
          maximumX(0.0f),
          minimumY(0.0f),
          maximumY(0.0f)
    {
    }

    float width() const
    {
        return maximumX - minimumX;
    }

    float height() const
    {
        return maximumY - minimumY;
    }
};


/*
 * ================================================================
 * SECTION 45 — MAP STATISTICS
 * ================================================================
 */

struct MapStatistics
{
    uint32_t unknownCells;

    uint32_t freeCells;

    uint32_t occupiedCells;

    uint32_t totalCells;

    float coveragePercentage;

    MapBounds boundaries;

    MapStatistics()
        : unknownCells(0),
          freeCells(0),
          occupiedCells(0),
          totalCells(0),
          coveragePercentage(0.0f),
          boundaries()
    {
    }
};


/*
 * ================================================================
 * SECTION 46 — OCCUPANCY GRID
 * ================================================================
 */

class OccupancyGrid
{
private:

    /*
     * Arduino Uno has limited SRAM.
     *
     * The configured map is therefore stored in a compact
     * dynamically allocated array.
     *
     * Each cell currently uses a compact representation.
     */

    MapCell* cells;

    int width;

    int height;

    float resolution;

    Vector2 origin;

    bool initialized;

public:

    OccupancyGrid()
        : cells(nullptr),
          width(0),
          height(0),
          resolution(0.1f),
          origin(),
          initialized(false)
    {
    }

    ~OccupancyGrid()
    {
        release();
    }

    bool initialize(
        int mapWidth,
        int mapHeight,
        float mapResolution,
        const Vector2& mapOrigin
    )
    {
        release();

        if (
            mapWidth <= 0 ||
            mapHeight <= 0
        )
        {
            return false;
        }

        if (
            mapResolution <= 0.0f
        )
        {
            return false;
        }

        long totalCells =
            static_cast<long>(
                mapWidth
            )
            *
            static_cast<long>(
                mapHeight
            );

        /*
         * Conservative Arduino memory guard.
         *
         * The Uno cannot safely hold a large desktop-sized map.
         */

        if (
            totalCells > 4000L
        )
        {
            return false;
        }

        cells =
            new MapCell[
                totalCells
            ];

        if (cells == nullptr)
        {
            return false;
        }

        width =
            mapWidth;

        height =
            mapHeight;

        resolution =
            mapResolution;

        origin =
            mapOrigin;

        initialized =
            true;

        clear();

        return true;
    }

    void release()
    {
        if (cells != nullptr)
        {
            delete[] cells;

            cells = nullptr;
        }

        width = 0;

        height = 0;

        initialized = false;
    }

    void clear()
    {
        if (!initialized)
        {
            return;
        }

        long total =
            static_cast<long>(
                width
            )
            *
            static_cast<long>(
                height
            );

        for (
            long index = 0;
            index < total;
            ++index
        )
        {
            cells[index].reset();
        }
    }

    bool isInitialized() const
    {
        return initialized;
    }

    int getWidth() const
    {
        return width;
    }

    int getHeight() const
    {
        return height;
    }

    float getResolution() const
    {
        return resolution;
    }

    Vector2 getOrigin() const
    {
        return origin;
    }

    bool isInside(
        int x,
        int y
    ) const
    {
        return
            x >= 0 &&
            y >= 0 &&
            x < width &&
            y < height;
    }

    int indexOf(
        int x,
        int y
    ) const
    {
        if (!isInside(x, y))
        {
            return -1;
        }

        return
            y * width +
            x;
    }

    MapCell* getCell(
        int x,
        int y
    )
    {
        int index =
            indexOf(
                x,
                y
            );

        if (index < 0)
        {
            return nullptr;
        }

        return &cells[index];
    }

    const MapCell* getCell(
        int x,
        int y
    ) const
    {
        int index =
            indexOf(
                x,
                y
            );

        if (index < 0)
        {
            return nullptr;
        }

        return &cells[index];
    }

    MapCellState getState(
        int x,
        int y
    ) const
    {
        const MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return MapCellState::UNKNOWN;
        }

        return static_cast<MapCellState>(
            cell->occupancy
        );
    }

    void setState(
        int x,
        int y,
        MapCellState state
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->occupancy =
            static_cast<int8_t>(
                state
            );

        cell->observations++;

        if (
            state ==
            MapCellState::FREE
        )
        {
            cell->freeObservations++;
        }

        if (
            state ==
            MapCellState::OCCUPIED
        )
        {
            cell->occupiedObservations++;
        }
    }

    void markFree(
        int x,
        int y
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->observations++;

        cell->freeObservations++;

        /*
         * A repeated free observation can override an
         * earlier unknown state.
         *
         * Occupied cells are deliberately not immediately
         * erased because an obstacle measurement should
         * require stronger evidence to disappear.
         */

        if (
            cell->occupiedObservations == 0 ||
            cell->freeObservations >
                cell->occupiedObservations + 2
        )
        {
            cell->occupancy =
                static_cast<int8_t>(
                    MapCellState::FREE
                );
        }
    }

    void markOccupied(
        int x,
        int y
    )
    {
        MapCell* cell =
            getCell(
                x,
                y
            );

        if (cell == nullptr)
        {
            return;
        }

        cell->observations++;

        cell->occupiedObservations++;

        /*
         * Occupied observations receive strong priority.
         */

        if (
            cell->occupiedObservations >=
            cell->freeObservations
        )
        {
            cell->occupancy =
                static_cast<int8_t>(
                    MapCellState::OCCUPIED
                );
        }
    }

    GridCoordinate worldToGrid(
        float worldX,
        float worldY
    ) const
    {
        float localX =
            worldX -
            origin.x;

        float localY =
            worldY -
            origin.y;

        int gridX =
            static_cast<int>(
                floor(
                    localX /
                    resolution
                )
            );

        int gridY =
            static_cast<int>(
                floor(
                    localY /
                    resolution
                )
            );

        return GridCoordinate(
            gridX,
            gridY
        );
    }

    Vector2 gridToWorld(
        int gridX,
        int gridY
    ) const
    {
        float worldX =
            origin.x
            +
            (
                gridX + 0.5f
            )
            *
            resolution;

        float worldY =
            origin.y
            +
            (
                gridY + 0.5f
            )
            *
            resolution;

        return Vector2(
            worldX,
            worldY
        );
    }

    MapStatistics getStatistics() const
    {
        MapStatistics statistics;

        if (!initialized)
        {
            return statistics;
        }

        statistics.totalCells =
            static_cast<uint32_t>(
                width * height
            );

        for (
            int y = 0;
            y < height;
            ++y
        )
        {
            for (
                int x = 0;
                x < width;
                ++x
            )
            {
                MapCellState state =
                    getState(
                        x,
                        y
                    );

                if (
                    state ==
                    MapCellState::UNKNOWN
                )
                {
                    statistics.unknownCells++;
                }
                else if (
                    state ==
                    MapCellState::FREE
                )
                {
                    statistics.freeCells++;
                }
                else if (
                    state ==
                    MapCellState::OCCUPIED
                )
                {
                    statistics.occupiedCells++;
                }
            }
        }

        uint32_t known =
            statistics.freeCells +
            statistics.occupiedCells;

        if (
            statistics.totalCells > 0
        )
        {
            statistics.coveragePercentage =
                (
                    static_cast<float>(
                        known
                    )
                    /
                    static_cast<float>(
                        statistics.totalCells
                    )
                )
                *
                100.0f;
        }

        statistics.boundaries =
            calculateBounds();

        return statistics;
    }

    MapBounds calculateBounds() const
    {
        MapBounds bounds;

        if (!initialized)
        {
            return bounds;
        }

        bounds.minimumX =
            origin.x;

        bounds.minimumY =
            origin.y;

        bounds.maximumX =
            origin.x +
            width * resolution;

        bounds.maximumY =
            origin.y +
            height * resolution;

        return bounds;
    }

    bool saveToSerial() const
    {
        if (!initialized)
        {
            return false;
        }

        Serial.println(
            F("MAP_BEGIN")
        );

        Serial.print(
            F("WIDTH=")
        );

        Serial.println(
            width
        );

        Serial.print(
            F("HEIGHT=")
        );

        Serial.println(
            height
        );

        Serial.print(
            F("RESOLUTION=")
        );

        Serial.println(
            resolution,
            4
        );

        for (
            int y = 0;
            y < height;
            ++y
        )
        {
            for (
                int x = 0;
                x < width;
                ++x
            )
            {
                MapCellState state =
                    getState(
                        x,
                        y
                    );

                char symbol = '?';

                if (
                    state ==
                    MapCellState::FREE
                )
                {
                    symbol = '.';
                }
                else if (
                    state ==
                    MapCellState::OCCUPIED
                )
                {
                    symbol = '#';
                }

                Serial.print(
                    symbol
                );
            }

            Serial.println();
        }

        Serial.println(
            F("MAP_END")
        );

        return true;
    }

    void printCompactMap(
        const Pose2D& robotPose
    ) const
    {
        if (!initialized)
        {
            Serial.println(
                F("Map not initialized.")
            );

            return;
        }

        /*
         * Printing the complete 80x80 map repeatedly would
         * consume a large amount of serial bandwidth.
         *
         * Therefore this diagnostic view displays a reduced
         * section around the robot.
         */

        GridCoordinate robotCell =
            worldToGrid(
                robotPose.x,
                robotPose.y
            );

        constexpr int VIEW_RADIUS = 12;

        int minimumX =
            robotCell.x -
            VIEW_RADIUS;

        int maximumX =
            robotCell.x +
            VIEW_RADIUS;

        int minimumY =
            robotCell.y -
            VIEW_RADIUS;

        int maximumY =
            robotCell.y +
            VIEW_RADIUS;

        Serial.println(
            F("---- LOCAL MAP ----")
        );

        for (
            int y = maximumY;
            y >= minimumY;
            --y
        )
        {
            for (
                int x = minimumX;
                x <= maximumX;
                ++x
            )
            {
                if (
                    x == robotCell.x &&
                    y == robotCell.y
                )
                {
                    Serial.print('R');

                    continue;
                }

                MapCellState state =
                    getState(
                        x,
                        y
                    );

                if (
                    state ==
                    MapCellState::UNKNOWN
                )
                {
                    Serial.print('?');
                }
                else if (
                    state ==
                    MapCellState::FREE
                )
                {
                    Serial.print('.');
                }
                else
                {
                    Serial.print('#');
                }
            }

            Serial.println();
        }

        Serial.println(
            F("--------------------")
        );
    }
};


/*
 * ================================================================
 * SECTION 47 — SENSOR POSE
 * ================================================================
 */

struct SensorPose
{
    float offsetX;

    float offsetY;

    float angleOffset;

    SensorPose()
        : offsetX(0.0f),
          offsetY(0.0f),
          angleOffset(0.0f)
    {
    }

    SensorPose(
        float x,
        float y,
        float angle
    )
        : offsetX(x),
          offsetY(y),
          angleOffset(angle)
    {
    }
};


/*
 * ================================================================
 * SECTION 48 — SENSOR PROJECTOR
 * ================================================================
 */

class SensorProjector
{
private:

    SensorPose frontPose;

    SensorPose leftPose;

    SensorPose rightPose;

public:

    SensorProjector()
        : frontPose(
              0.08f,
              0.0f,
              0.0f
          ),
          leftPose(
              0.0f,
              0.06f,
              PI_VALUE / 2.0f
          ),
          rightPose(
              0.0f,
              -0.06f,
              -PI_VALUE / 2.0f
          )
    {
    }

    void setFrontPose(
        const SensorPose& pose
    )
    {
        frontPose =
            pose;
    }

    void setLeftPose(
        const SensorPose& pose
    )
    {
        leftPose =
            pose;
    }

    void setRightPose(
        const SensorPose& pose
    )
    {
        rightPose =
            pose;
    }

    Vector2 transformSensorOrigin(
        const Pose2D& robotPose,
        const SensorPose& sensorPose
    ) const
    {
        /*
         * Robot-local sensor position:
         *
         * [offsetX]
         * [offsetY]
         *
         * is rotated by robot heading theta.
         */

        float cosTheta =
            cos(
                robotPose.theta
            );

        float sinTheta =
            sin(
                robotPose.theta
            );

        float worldX =
            robotPose.x
            +
            sensorPose.offsetX *
            cosTheta
            -
            sensorPose.offsetY *
            sinTheta;

        float worldY =
            robotPose.y
            +
            sensorPose.offsetX *
            sinTheta
            +
            sensorPose.offsetY *
            cosTheta;

        return Vector2(
            worldX,
            worldY
        );
    }

    SensorRay project(
        const Pose2D& robotPose,
        const SensorPose& sensorPose,
        float distance
    ) const
    {
        SensorRay ray;

        if (
            distance <= 0.0f
        )
        {
            return ray;
        }

        ray.origin =
            transformSensorOrigin(
                robotPose,
                sensorPose
            );

        ray.angle =
            normalizeAngle(
                robotPose.theta
                +
                sensorPose.angleOffset
            );

        ray.distance =
            distance;

        ray.endpoint.x =
            ray.origin.x
            +
            cos(ray.angle)
            *
            distance;

        ray.endpoint.y =
            ray.origin.y
            +
            sin(ray.angle)
            *
            distance;

        ray.valid = true;

        return ray;
    }

    SensorRay projectFront(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            frontPose,
            distance
        );
    }

    SensorRay projectLeft(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            leftPose,
            distance
        );
    }

    SensorRay projectRight(
        const Pose2D& robotPose,
        float distance
    ) const
    {
        return project(
            robotPose,
            rightPose,
            distance
        );
    }
};


/*
 * ================================================================
 * SECTION 49 — RAY CASTING RESULT
 * ================================================================
 */

struct RayCastResult
{
    uint16_t freeCellsUpdated;

    bool obstacleMarked;

    bool terminatedByBoundary;

    bool terminatedByMaximumRange;

    RayCastResult()
        : freeCellsUpdated(0),
          obstacleMarked(false),
          terminatedByBoundary(false),
          terminatedByMaximumRange(false)
    {
    }
};


/*
 * ================================================================
 * SECTION 50 — RAY CASTER
 * ================================================================
 */

class RayCaster
{
private:

    OccupancyGrid* map;

public:

    RayCaster()
        : map(nullptr)
    {
    }

    void attachMap(
        OccupancyGrid* occupancyGrid
    )
    {
        map =
            occupancyGrid;
    }

    /*
     * Integer grid traversal based on the ray's world
     * endpoints.
     *
     * The implementation walks through the line using
     * incremental steps and marks intermediate cells free.
     */

    RayCastResult castRay(
        const SensorRay& ray,
        bool obstacleDetectedAtEndpoint
    )
    {
        RayCastResult result;

        if (
            map == nullptr ||
            !map->isInitialized()
        )
        {
            return result;
        }

        if (!ray.valid)
        {
            return result;
        }

        GridCoordinate start =
            map->worldToGrid(
                ray.origin.x,
                ray.origin.y
            );

        GridCoordinate end =
            map->worldToGrid(
                ray.endpoint.x,
                ray.endpoint.y
            );

        /*
         * Number of interpolation steps.
         *
         * At least one step per map cell crossed.
         */

        int dx =
            abs(
                end.x -
                start.x
            );

        int dy =
            abs(
                end.y -
                start.y
            );

        int steps =
            max(
                dx,
                dy
            );

        if (steps <= 0)
        {
            if (
                obstacleDetectedAtEndpoint
            )
            {
                map->markOccupied(
                    start.x,
                    start.y
                );

                result.obstacleMarked =
                    true;
            }

            return result;
        }

        int previousX =
            start.x;

        int previousY =
            start.y;

        for (
            int step = 0;
            step <= steps;
            ++step
        )
        {
            float ratio =
                static_cast<float>(
                    step
                )
                /
                static_cast<float>(
                    steps
                );

            float worldX =
                ray.origin.x
                +
                (
                    ray.endpoint.x -
                    ray.origin.x
                )
                *
                ratio;

            float worldY =
                ray.origin.y
                +
                (
                    ray.endpoint.y -
                    ray.origin.y
                )
                *
                ratio;

            GridCoordinate current =
                map->worldToGrid(
                    worldX,
                    worldY
                );

            if (
                current.x ==
                    previousX &&
                current.y ==
                    previousY
            )
            {
                continue;
            }

            previousX =
                current.x;

            previousY =
                current.y;

            if (
                !map->isInside(
                    current.x,
                    current.y
                )
            )
            {
                result.terminatedByBoundary =
                    true;

                break;
            }

            bool isLastCell =
                (
                    step >=
                    steps - 1
                );

            if (
                isLastCell &&
                obstacleDetectedAtEndpoint
            )
            {
                map->markOccupied(
                    current.x,
                    current.y
                );

                result.obstacleMarked =
                    true;
            }
            else
            {
                map->markFree(
                    current.x,
                    current.y
                );

                result.freeCellsUpdated++;
            }
        }

        return result;
    }
};


/*
 * ================================================================
 * SECTION 51 — MAPPING SENSOR TYPE
 * ================================================================
 */

enum class MappingSensorType
{
    FRONT,
    LEFT,
    RIGHT
};


/*
 * ================================================================
 * SECTION 52 — MAP UPDATE REPORT
 * ================================================================
 */

struct MapUpdateReport
{
    uint16_t raysProcessed;

    uint16_t freeCellsUpdated;

    uint16_t occupiedCellsUpdated;

    uint16_t invalidMeasurements;

    uint16_t boundaryTerminations;

    MapUpdateReport()
        : raysProcessed(0),
          freeCellsUpdated(0),
          occupiedCellsUpdated(0),
          invalidMeasurements(0),
          boundaryTerminations(0)
    {
    }
};


/*
 * ================================================================
 * SECTION 53 — MAPPING ENGINE
 * ================================================================
 */

class MappingEngine
{
private:

    OccupancyGrid occupancyGrid;

    SensorProjector sensorProjector;

    RayCaster rayCaster;

    float maximumRange;

    float obstacleThreshold;

    bool initialized;

public:

    MappingEngine()
        : maximumRange(300.0f),
          obstacleThreshold(290.0f),
          initialized(false)
    {
    }

    bool initialize(
        int width,
        int height,
        float resolution,
        const Vector2& origin
    )
    {
        bool result =
            occupancyGrid.initialize(
                width,
                height,
                resolution,
                origin
            );

        if (!result)
        {
            initialized =
                false;

            return false;
        }

        rayCaster.attachMap(
            &occupancyGrid
        );

        initialized =
            true;

        return true;
    }

    bool isInitialized() const
    {
        return initialized;
    }

    OccupancyGrid& getMap()
    {
        return occupancyGrid;
    }

    const OccupancyGrid& getMap() const
    {
        return occupancyGrid;
    }

    SensorProjector& getSensorProjector()
    {
        return sensorProjector;
    }

    void setMaximumRange(
        float range
    )
    {
        maximumRange =
            max(
                0.1f,
                range
            );
    }

    void setObstacleThreshold(
        float threshold
    )
    {
        obstacleThreshold =
            clampFloat(
                threshold,
                0.1f,
                maximumRange
            );
    }

    bool isValidMeasurement(
        float distance
    ) const
    {
        if (
            isnan(distance) ||
            isinf(distance)
        )
        {
            return false;
        }

        if (
            distance <= 0.0f
        )
        {
            return false;
        }

        if (
            distance > maximumRange
        )
        {
            return false;
        }

        return true;
    }

    bool isObstacleMeasurement(
        float distance
    ) const
    {
        return
            distance <
            obstacleThreshold;
    }

    MapUpdateReport update(
        const Pose2D& robotPose,
        const DistanceSensorArray& sensors
    )
    {
        MapUpdateReport report;

        if (!initialized)
        {
            return report;
        }

        /*
         * FRONT SENSOR
         */

        if (sensors.frontValid)
        {
            if (
                isValidMeasurement(
                    sensors.front
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectFront(
                        robotPose,
                        sensors.front
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.front
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        /*
         * LEFT SENSOR
         */

        if (sensors.leftValid)
        {
            if (
                isValidMeasurement(
                    sensors.left
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectLeft(
                        robotPose,
                        sensors.left
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.left
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        /*
         * RIGHT SENSOR
         */

        if (sensors.rightValid)
        {
            if (
                isValidMeasurement(
                    sensors.right
                )
            )
            {
                SensorRay ray =
                    sensorProjector.projectRight(
                        robotPose,
                        sensors.right
                    );

                bool obstacle =
                    isObstacleMeasurement(
                        sensors.right
                    );

                RayCastResult result =
                    rayCaster.castRay(
                        ray,
                        obstacle
                    );

                report.raysProcessed++;

                report.freeCellsUpdated +=
                    result.freeCellsUpdated;

                if (
                    result.obstacleMarked
                )
                {
                    report.occupiedCellsUpdated++;
                }

                if (
                    result.terminatedByBoundary
                )
                {
                    report.boundaryTerminations++;
                }
            }
            else
            {
                report.invalidMeasurements++;
            }
        }
        else
        {
            report.invalidMeasurements++;
        }

        return report;
    }

    void clear()
    {
        occupancyGrid.clear();
    }

    MapStatistics statistics() const
    {
        return occupancyGrid.getStatistics();
    }

    void printStatistics() const
    {
        MapStatistics stats =
            statistics();

        Serial.println(
            F("========== MAP STATISTICS ==========")
        );

        Serial.print(
            F("Total cells: ")
        );

        Serial.println(
            stats.totalCells
        );

        Serial.print(
            F("Unknown cells: ")
        );

        Serial.println(
            stats.unknownCells
        );

        Serial.print(
            F("Free cells: ")
        );

        Serial.println(
            stats.freeCells
        );

        Serial.print(
            F("Occupied cells: ")
        );

        Serial.println(
            stats.occupiedCells
        );

        Serial.print(
            F("Coverage: ")
        );

        Serial.print(
            stats.coveragePercentage,
            2
        );

        Serial.println(
            F("%")
        );

        Serial.print(
            F("Map width: ")
        );

        Serial.print(
            stats.boundaries.width(),
            2
        );

        Serial.println(
            F(" m")
        );

        Serial.print(
            F("Map height: ")
        );

        Serial.print(
            stats.boundaries.height(),
            2
        );

        Serial.println(
            F(" m")
        );

        Serial.println(
            F("====================================")
        );
    }

    void printLocalMap(
        const Pose2D& robotPose
    ) const
    {
        occupancyGrid.printCompactMap(
            robotPose
        );
    }

    bool saveMap()
    {
        return occupancyGrid.saveToSerial();
    }
};


/*
 * ================================================================
 * SECTION 54 — GLOBAL MAPPING ENGINE
 * ================================================================
 */

MappingEngine mappingEngine;


/*
 * ================================================================
 * SECTION 55 — MAP INITIALIZATION
 * ================================================================
 */

void initializeMapping()
{
    /*
     * Map origin is positioned so that the initial robot
     * begins near the center of the map.
     */

    float mapWidthMeters =
        config.mapWidth *
        config.mapResolution;

    float mapHeightMeters =
        config.mapHeight *
        config.mapResolution;

    Vector2 origin(
        -mapWidthMeters / 2.0f,
        -mapHeightMeters / 2.0f
    );

    bool initialized =
        mappingEngine.initialize(
            config.mapWidth,
            config.mapHeight,
            config.mapResolution,
            origin
        );

    if (initialized)
    {
        Serial.println(
            F("Mapping engine: OK")
        );
    }
    else
    {
        Serial.println(
            F("Mapping engine: FAILED")
        );
    }

    mappingEngine.setMaximumRange(
        config.ultrasonicMaximumDistance
    );

    /*
     * Measurements closer than this threshold are treated
     * as obstacle detections.
     */

    mappingEngine.setObstacleThreshold(
        280.0f
    );
}


/*
 * ================================================================
 * SECTION 56 — MAPPING UPDATE LOOP
 * ================================================================
 */

MapUpdateReport lastMapReport;

void updateMapping()
{
    unsigned long currentTime =
        millis();

    float period =
        1000.0f /
        config.mappingFrequency;

    if (
        (
            currentTime -
            previousMappingTime
        )
        <
        period
    )
    {
        return;
    }

    previousMappingTime =
        currentTime;

    if (
        !mappingEngine.isInitialized()
    )
    {
        return;
    }

    lastMapReport =
        mappingEngine.update(
            robotState.pose,
            distanceSensors
        );
}


/*
 * ================================================================
 * SECTION 57 — MAP DIAGNOSTIC COMMANDS
 * ================================================================
 */

void processMapCommand(
    const String& command
)
{
    if (
        command == "map"
    )
    {
        mappingEngine.printStatistics();

        return;
    }

    if (
        command == "map_view"
    )
    {
        mappingEngine.printLocalMap(
            robotState.pose
        );

        return;
    }

    if (
        command == "save_map"
    )
    {
        bool success =
            mappingEngine.saveMap();

        if (success)
        {
            Serial.println(
                F("Map exported through Serial.")
            );
        }
        else
        {
            Serial.println(
                F("Map export failed.")
            );
        }

        return;
    }

    if (
        command == "clear_map"
    )
    {
        mappingEngine.clear();

        Serial.println(
            F("Map cleared.")
        );

        return;
    }
}


/*
 * ================================================================
 * SECTION 58 — EXTENDED COMMAND ROUTER
 * ================================================================
 */

void processExtendedCommand(
    String command
)
{
    command.trim();

    command.toLowerCase();

    if (
        command == "map" ||
        command == "map_view" ||
        command == "save_map" ||
        command == "clear_map"
    )
    {
        processMapCommand(
            command
        );

        return;
    }

    processCommand(
        command
    );
}


/*
 * ================================================================
 * SECTION 59 — MAPPING-AWARE COMMAND INTERFACE
 * ================================================================
 */

void updateMappingCommandInterface()
{
    static String commandBuffer;

    while (
        Serial.available() > 0
    )
    {
        char character =
            Serial.read();

        if (
            character == '\n' ||
            character == '\r'
        )
        {
            if (
                commandBuffer.length()
                > 0
            )
            {
                processExtendedCommand(
                    commandBuffer
                );

                commandBuffer =
                    "";
            }
        }
        else
        {
            if (
                commandBuffer.length()
                < 80
            )
            {
                commandBuffer +=
                    character;
            }
        }
    }
}


/*
 * ================================================================
 * SECTION 60 — MAP COVERAGE CHECK
 * ================================================================
 */

float getMapCoverage()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return 0.0f;
    }

    MapStatistics statistics =
        mappingEngine.statistics();

    return
        statistics.coveragePercentage;
}


/*
 * ================================================================
 * SECTION 61 — WORLD TO MAP HELPER
 * ================================================================
 */

GridCoordinate worldPositionToMapCell(
    float x,
    float y
)
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return GridCoordinate(
            0,
            0
        );
    }

    return
        mappingEngine
            .getMap()
            .worldToGrid(
                x,
                y
            );
}


/*
 * ================================================================
 * SECTION 62 — MAP CELL QUERY
 * ================================================================
 */

MapCellState getMapCellState(
    int x,
    int y
)
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return MapCellState::UNKNOWN;
    }

    return
        mappingEngine
            .getMap()
            .getState(
                x,
                y
            );
}


/*
 * ================================================================
 * SECTION 63 — OBSTACLE QUERY IN WORLD SPACE
 * ================================================================
 */

bool isWorldPositionOccupied(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::OCCUPIED;
}


/*
 * ================================================================
 * SECTION 64 — FREE-SPACE QUERY
 * ================================================================
 */

bool isWorldPositionFree(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::FREE;
}


/*
 * ================================================================
 * SECTION 65 — UNKNOWN-SPACE QUERY
 * ================================================================
 */

bool isWorldPositionUnknown(
    float x,
    float y
)
{
    GridCoordinate cell =
        worldPositionToMapCell(
            x,
            y
        );

    MapCellState state =
        getMapCellState(
            cell.x,
            cell.y
        );

    return
        state ==
        MapCellState::UNKNOWN;
}


/*
 * ================================================================
 * SECTION 66 — SENSOR RAY DEBUGGING
 * ================================================================
 */

void printSensorRay(
    const SensorRay& ray,
    const char* name
)
{
    Serial.print(
        name
    );

    Serial.print(
        F(" origin=(")
    );

    Serial.print(
        ray.origin.x,
        3
    );

    Serial.print(
        F(",")
    );

    Serial.print(
        ray.origin.y,
        3
    );

    Serial.print(
        F(") endpoint=(")
    );

    Serial.print(
        ray.endpoint.x,
        3
    );

    Serial.print(
        F(",")
    );

    Serial.print(
        ray.endpoint.y,
        3
    );

    Serial.print(
        F(") distance=")
    );

    Serial.print(
        ray.distance,
        2
    );

    Serial.print(
        F(" angle=")
    );

    Serial.print(
        radiansToDegrees(
            ray.angle
        ),
        2
    );

    Serial.println(
        F(" deg")
    );
}


/*
 * ================================================================
 * SECTION 67 — SENSOR PROJECTION DIAGNOSTICS
 * ================================================================
 */

void printSensorProjectionDiagnostics()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        Serial.println(
            F("Mapping engine not initialized.")
        );

        return;
    }

    SensorProjector& projector =
        mappingEngine
            .getSensorProjector();

    SensorRay frontRay =
        projector.projectFront(
            robotState.pose,
            distanceSensors.front
        );

    SensorRay leftRay =
        projector.projectLeft(
            robotState.pose,
            distanceSensors.left
        );

    SensorRay rightRay =
        projector.projectRight(
            robotState.pose,
            distanceSensors.right
        );

    Serial.println(
        F("===== SENSOR PROJECTION =====")
    );

    printSensorRay(
        frontRay,
        "FRONT"
    );

    printSensorRay(
        leftRay,
        "LEFT"
    );

    printSensorRay(
        rightRay,
        "RIGHT"
    );

    Serial.println(
        F("=============================")
    );
}


/*
 * ================================================================
 * SECTION 68 — MAP UPDATE DIAGNOSTICS
 * ================================================================
 */

void printLastMapUpdate()
{
    Serial.println(
        F("===== LAST MAP UPDATE =====")
    );

    Serial.print(
        F("Rays: ")
    );

    Serial.println(
        lastMapReport.raysProcessed
    );

    Serial.print(
        F("Free cells: ")
    );

    Serial.println(
        lastMapReport.freeCellsUpdated
    );

    Serial.print(
        F("Occupied cells: ")
    );

    Serial.println(
        lastMapReport.occupiedCellsUpdated
    );

    Serial.print(
        F("Invalid measurements: ")
    );

    Serial.println(
        lastMapReport.invalidMeasurements
    );

    Serial.print(
        F("Boundary terminations: ")
    );

    Serial.println(
        lastMapReport.boundaryTerminations
    );

    Serial.println(
        F("===========================")
    );
}


/*
 * ================================================================
 * SECTION 69 — MAPPING STATUS
 * ================================================================
 */

void printMappingStatus()
{
    Serial.println(
        F("========== MAPPING STATUS ==========")
    );

    Serial.print(
        F("Initialized: ")
    );

    Serial.println(
        mappingEngine.isInitialized()
            ? F("YES")
            : F("NO")
    );

    Serial.print(
        F("Map width: ")
    );

    Serial.println(
        mappingEngine
            .getMap()
            .getWidth()
    );

    Serial.print(
        F("Map height: ")
    );

    Serial.println(
        mappingEngine
            .getMap()
            .getHeight()
    );

    Serial.print(
        F("Resolution: ")
    );

    Serial.print(
        mappingEngine
            .getMap()
            .getResolution(),
        3
    );

    Serial.println(
        F(" m/cell")
    );

    Serial.print(
        F("Coverage: ")
    );

    Serial.print(
        getMapCoverage(),
        2
    );

    Serial.println(
        F("%")
    );

    Serial.println(
        F("====================================")
    );
}


/*
 * ================================================================
 * SECTION 70 — MAPPING TEST
 * ================================================================
 */

bool runMappingBasicTest()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return false;
    }

    OccupancyGrid& map =
        mappingEngine.getMap();

    /*
     * Select a central cell.
     */

    int centerX =
        map.getWidth() / 2;

    int centerY =
        map.getHeight() / 2;

    map.markFree(
        centerX,
        centerY
    );

    map.markFree(
        centerX + 1,
        centerY
    );

    map.markFree(
        centerX,
        centerY + 1
    );

    map.markOccupied(
        centerX + 2,
        centerY
    );

    MapCellState freeState =
        map.getState(
            centerX,
            centerY
        );

    MapCellState occupiedState =
        map.getState(
            centerX + 2,
            centerY
        );

    bool freeCorrect =
        freeState ==
        MapCellState::FREE;

    bool occupiedCorrect =
        occupiedState ==
        MapCellState::OCCUPIED;

    return
        freeCorrect &&
        occupiedCorrect;
}


/*
 * ================================================================
 * SECTION 71 — RAY CASTING TEST
 * ================================================================
 */

bool runRayCastingTest()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return false;
    }

    Pose2D testPose;

    testPose.x = 0.0f;

    testPose.y = 0.0f;

    testPose.theta = 0.0f;

    SensorRay ray =
        mappingEngine
            .getSensorProjector()
            .projectFront(
                testPose,
                1.0f
            );

    RayCaster caster;

    caster.attachMap(
        &mappingEngine.getMap()
    );

    RayCastResult result =
        caster.castRay(
            ray,
            true
        );

    return
        result.freeCellsUpdated > 0
        ||
        result.obstacleMarked;
}


/*
 * ================================================================
 * SECTION 72 — MAP TEST SUITE
 * ================================================================
 */

void runMappingTests()
{
    Serial.println(
        F("")
    );

    Serial.println(
        F("========== MAPPING TESTS ==========")
    );

    bool gridTest =
        runMappingBasicTest();

    Serial.print(
        F("Occupancy Grid Test: ")
    );

    Serial.println(
        gridTest
            ? F("PASS")
            : F("FAIL")
    );

    bool rayTest =
        runRayCastingTest();

    Serial.print(
        F("Ray Casting Test: ")
    );

    Serial.println(
        rayTest
            ? F("PASS")
            : F("FAIL")
    );

    Serial.println(
        F("===================================")
    );

    Serial.println();
}


/*
 * ================================================================
 * SECTION 73 — INITIALIZE PART 2
 * ================================================================
 */

void initializeMappingSubsystem()
{
    initializeMapping();

    if (
        mappingEngine.isInitialized()
    )
    {
        runMappingTests();
    }
}


/*
 * ================================================================
 * SECTION 74 — MAPPING LOOP INTEGRATION
 * ================================================================
 */

void runMappingSubsystem()
{
    if (
        !mappingEngine.isInitialized()
    )
    {
        return;
    }

    updateMapping();
}


/*
 * ================================================================
 * SECTION 75 — END OF PART 2
 * ================================================================
 *
 * IMPLEMENTED:
 *
 *  [x] Occupancy grid
 *  [x] Unknown/free/occupied states
 *  [x] Configurable map resolution
 *  [x] Configurable map dimensions
 *  [x] World -> grid conversion
 *  [x] Grid -> world conversion
 *  [x] Sensor coordinate transformation
 *  [x] Sensor ray generation
 *  [x] Ray traversal
 *  [x] Free-space marking
 *  [x] Occupied-cell marking
 *  [x] Map coverage calculation
 *  [x] Map statistics
 *  [x] Map boundaries
 *  [x] Local map visualization through Serial
 *  [x] Map export through Serial
 *  [x] Map clearing
 *  [x] Mapping diagnostics
 *  [x] Mapping tests
 *
 * NEXT:
 *
 *  PART 3
 *      A* PATH PLANNING
 *      PATH VALIDATION
 *      PATH SMOOTHING
 *      GRID NAVIGATION
 *
 * ================================================================
 */